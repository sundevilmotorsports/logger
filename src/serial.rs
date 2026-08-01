use crate::configuration::{Configuration, CONFIGURATION};
use crate::state::State;
use crate::usb_hs::UsbHsCdc;
use crate::{logging, sd_fake};
use esp_idf_svc::hal::delay::{self, FreeRtos};
use esp_idf_svc::sys::{esp_restart, esp_timer_get_time};
use serde::Deserialize;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::time::Duration;

const MAX_PAYLOAD: usize = 16 * 1024;
const CHANNEL_CAPACITY: usize = 4;

/// Bytes per `LogChunk` response; the client requests successive offsets
/// until it gets back fewer than this many bytes.
const LOG_CHUNK_LEN: usize = 512;

/// Set by `Command::Reboot`; checked after its ack is sent so the client
/// gets a clean response before the device actually goes down.
static REBOOT_REQUESTED: AtomicBool = AtomicBool::new(false);

#[derive(Deserialize)]
#[serde(tag = "cmd", rename_all = "snake_case")]
enum Command {
    Ping,
    Version,
    Status,
    GetConfig,
    SetConfig { args: Configuration },
    Frames,
    Uptime,
    Gps,
    Imu,
    Resources,
    ListLogs,
    LogChunk { name: String, offset: u64 },
    LogStatus,
    SetLogging { active: bool },
    NextLog,
    Reboot,
}

/// Spawns the USB reader thread and the command-dispatch thread that
/// processes what it receives, connected by a pair of channels.
pub fn spawn(driver: UsbHsCdc, state: Arc<State>) -> bool {
    let (cmd_tx, cmd_rx) = mpsc::sync_channel::<String>(CHANNEL_CAPACITY);
    let (resp_tx, resp_rx) = mpsc::sync_channel::<String>(CHANNEL_CAPACITY);

    let reader_spawned = std::thread::Builder::new()
        .stack_size(8192)
        .spawn(move || reader_thread(driver, cmd_tx, resp_rx))
        .inspect_err(|e| log::error!("serial reader thread spawn failed: {e:?}"))
        .is_ok();
    if !reader_spawned {
        return false;
    }

    std::thread::Builder::new()
        .spawn(move || dispatch_thread(state, cmd_rx, resp_tx))
        .inspect_err(|e| log::error!("serial dispatch thread spawn failed: {e:?}"))
        .is_ok()
}

fn dispatch_thread(state: Arc<State>, cmd_rx: mpsc::Receiver<String>, resp_tx: mpsc::SyncSender<String>) {
    while let Ok(payload) = cmd_rx.recv() {
        let response = handle_command(&payload, &state);
        if resp_tx.send(response).is_err() {
            break; // reader thread gone
        }

        if REBOOT_REQUESTED.load(Ordering::Relaxed) {
            std::thread::sleep(Duration::from_millis(200)); // give the ack time to actually flush over USB
            unsafe { esp_restart() };
        }
    }
}

fn hex_encode(bytes: &[u8]) -> String {
    use std::fmt::Write;
    let mut out = String::with_capacity(bytes.len() * 2);
    for b in bytes {
        let _ = write!(out, "{b:02x}");
    }
    out
}

fn handle_command(payload: &str, state: &State) -> String {
    let ok =
        |data: serde_json::Value| serde_json::json!({"ok": true, "data": data}).to_string() + "\n";
    let err = |msg: String| serde_json::json!({"ok": false, "error": msg}).to_string() + "\n";

    let cmd: Command = match serde_json::from_str(payload) {
        Ok(c) => c,
        Err(e) => return err(format!("invalid command: {e}")),
    };

    match cmd {
        Command::Ping => ok(serde_json::json!("pong")),

        Command::Version => ok(serde_json::json!({ "version": env!("CARGO_PKG_VERSION") })),

        Command::Status => {
            let loaded = !CONFIGURATION.lock().can_devices.is_empty();
            ok(serde_json::json!({
                "config_loaded": loaded,
                "subsystems": state.status.to_json(),
            }))
        }

        Command::GetConfig => match Configuration::json() {
            Ok(j) => {
                let v: serde_json::Value = serde_json::from_str(&j).unwrap_or_default();
                ok(v)
            }
            Err(e) => err(e.to_string()),
        },

        Command::SetConfig { args } => {
            let mut guard = CONFIGURATION.lock();
            guard.can_devices = args.can_devices;
            guard.adc_channels = args.adc_channels;
            drop(guard);
            ok(serde_json::Value::Null)
        }

        Command::Frames => {
            let snapshot: Vec<_> = {
                let signals = state.sensors.can.lock();
                signals
                    .iter()
                    .map(|(name, bytes)| serde_json::json!({"name": name, "bytes": bytes}))
                    .collect()
            };
            ok(serde_json::json!(snapshot))
        }

        Command::Uptime => {
            let time = unsafe { esp_timer_get_time() as f64 / 1_000_000.0 } as u64;
            ok(serde_json::json!({ "uptime_seconds": time }))
        }

        Command::Gps => match &*state.sensors.gps.lock() {
            Some(fix) => ok(serde_json::to_value(fix).unwrap_or_default()),
            None => err("no fix".into()),
        },

        Command::Imu => match &*state.sensors.imu.lock() {
            Some(imu) => ok(serde_json::to_value(imu).unwrap_or_default()),
            None => err("no imu".into()),
        },

        Command::Resources => ok(serde_json::to_value(state.resources.lock().sample()).unwrap_or_default()),

        Command::ListLogs => {
            let sd = sd_fake::SD.lock();
            let logs: Vec<_> = sd
                .list_logs()
                .iter()
                .map(|name| serde_json::json!({"name": name, "size": sd.file_size(name)}))
                .collect();
            ok(serde_json::json!(logs))
        }

        Command::LogChunk { name, offset } => {
            let sd = sd_fake::SD.lock();
            match sd.read_chunk(&name, offset, LOG_CHUNK_LEN) {
                // Hex-encoded since the log is binary now, not ASCII.
                Some(chunk) => ok(serde_json::json!({
                    "data": hex_encode(chunk),
                    "eof": chunk.len() < LOG_CHUNK_LEN,
                })),
                None => err(format!("no such log: {name}")),
            }
        }

        Command::LogStatus => {
            let current = sd_fake::SD.lock().current_name();
            ok(serde_json::json!({
                "active": logging::ACTIVE.load(std::sync::atomic::Ordering::Relaxed),
                "current": current,
            }))
        }

        Command::SetLogging { active } => {
            logging::ACTIVE.store(active, std::sync::atomic::Ordering::Relaxed);
            ok(serde_json::Value::Null)
        }

        Command::NextLog => {
            sd_fake::SD.lock().next_log().ok();
            ok(serde_json::Value::Null)
        }

        Command::Reboot => {
            REBOOT_REQUESTED.store(true, Ordering::Relaxed);
            ok(serde_json::Value::Null)
        }
    }
}

fn reader_thread(mut driver: UsbHsCdc, cmd_tx: mpsc::SyncSender<String>, resp_rx: mpsc::Receiver<String>) {
    use embedded_hal::delay::DelayNs;

    let mut buf = Vec::<u8>::new();
    let mut tmp = [0u8; 64];

    loop {
        while let Ok(resp) = resp_rx.try_recv() {
            let _ = driver.write(resp.as_bytes(), delay::BLOCK);
        }

        let n = driver.read(&mut tmp).unwrap_or(0);
        if n == 0 {
            FreeRtos.delay_ms(10);
        }
        for &b in &tmp[..n] {
            if b == b'\n' {
                let payload = String::from_utf8_lossy(&buf).trim().to_string();
                buf.clear();
                if !payload.is_empty() {
                    if cmd_tx.try_send(payload).is_err() {
                        log::warn!("serial: command channel full, dropping payload");
                    }
                }
            } else if buf.len() < MAX_PAYLOAD {
                buf.push(b);
            } else {
                log::warn!("serial: payload exceeded {MAX_PAYLOAD} bytes, discarding");
                buf.clear();
            }
        }
    }
}
