//! Request/response protocol with the desktop client over USB CDC. Decodes
//! commands (get/set config, list and download logs, start/stop logging,
//! reboot, status) and replies over the same [`UsbHsCdc`] link. Log downloads
//! are chunked: the client asks for offsets until it gets back a short read.

use crate::configuration::{Configuration, CONFIGURATION};
use crate::sd::SdCard;
use crate::state::{OtaProgress, OtaRequest, Staged, State};
use crate::usb_hs::UsbHsCdc;
use esp_idf_svc::hal::delay::{self, FreeRtos};
use esp_idf_svc::ota::{EspOta, EspOtaUpdate};
use esp_idf_svc::sys::{esp_restart, esp_timer_get_time};
use sdm_utils as sdm;
use serde::Deserialize;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::time::Duration;

const MAX_PAYLOAD: usize = 16 * 1024;
const CHANNEL_CAPACITY: usize = 4;

/// Bytes per `LogChunk` response; the client requests offsets until it gets back fewer.
const LOG_CHUNK_LEN: usize = 512;

const OTA_STAGE_CAP: usize = 16 * 1024;
const OTA_STAGE_WAIT_MS: u32 = 5_000;

/// `OtaProgress::result` for a self-update that stalled waiting for more bytes
const OTA_SELF_STARVED: u8 = 0xF4;
/// Give the client a moment to poll a successful result before we reboot
const OTA_SELF_REBOOT_DELAY_MS: u64 = 1_500;
const OTA_SELF_IDLE_TIMEOUT_MS: u32 = 10_000;

/// Set by `Command::Reboot`; checked after the ack is sent so the client gets a clean response first.
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
    CanNodes,
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
    OtaUpload { offset: u64, data: String },
    OtaFlash { node: u8, size: u32, crc: u32 },
    OtaStatus,
}

/// Spawns the USB reader thread and the command-dispatch thread, linked by a channel pair.
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
        .stack_size(8192)
        .spawn(move || dispatch_thread(state, cmd_rx, resp_tx))
        .inspect_err(|e| log::error!("serial dispatch thread spawn failed: {e:?}"))
        .is_ok()
}

fn dispatch_thread(
    state: Arc<State>,
    cmd_rx: mpsc::Receiver<String>,
    resp_tx: mpsc::SyncSender<String>,
) {
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

fn handle_command(payload: &str, state: &Arc<State>) -> String {
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
            state.logging.config_changed.store(true, Ordering::Relaxed);
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

        Command::CanNodes => {
            let now = unsafe { esp_timer_get_time() };
            let nodes: Vec<_> = state
                .sensors
                .can_nodes
                .lock()
                .iter()
                .map(|(node, n)| {
                    serde_json::json!({
                        "node": node,
                        "type": n.device_type,
                        "age_ms": (now - n.last_seen_us) / 1000,
                    })
                })
                .collect();
            ok(serde_json::json!(nodes))
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

        Command::Resources => {
            ok(serde_json::to_value(state.resources.lock().sample()).unwrap_or_default())
        }

        Command::ListLogs => {
            let logs: Vec<_> = SdCard::list_logs()
                .iter()
                .map(|name| serde_json::json!({"name": name, "size": SdCard::file_size(name)}))
                .collect();
            ok(serde_json::json!(logs))
        }

        Command::LogChunk { name, offset } => {
            match SdCard::read_chunk(&name, offset, LOG_CHUNK_LEN) {
                // Hex-encoded since the log is binary now, not ASCII.
                Some(chunk) => ok(serde_json::json!({
                    "data": hex_encode(&chunk),
                    "eof": chunk.len() < LOG_CHUNK_LEN,
                })),
                None => err(format!("no such log: {name}")),
            }
        }

        Command::LogStatus => {
            let current = SdCard::current_name().unwrap_or_default();
            ok(serde_json::json!({
                "active": state.logging.active.load(Ordering::Relaxed),
                "current": current,
            }))
        }

        Command::SetLogging { active } => {
            state.logging.active.store(active, Ordering::Relaxed);
            ok(serde_json::Value::Null)
        }

        Command::NextLog => {
            SdCard::next_log().ok();
            ok(serde_json::Value::Null)
        }

        Command::Reboot => {
            REBOOT_REQUESTED.store(true, Ordering::Relaxed);
            ok(serde_json::Value::Null)
        }

        Command::OtaUpload { offset, data } => {
            let bytes = match hex_decode(&data) {
                Ok(b) => b,
                Err(e) => return err(e),
            };
            // Feed the buffer waiting if the CAN thread hasnt drained enough yet
            let mut waited_ms = 0u32;
            loop {
                if let Some(code) = state.ota.progress.lock().result {
                    return err(format!("ota aborted (code {code})"));
                }
                {
                    let mut s = state.ota.staged.lock();
                    if offset != s.end() as u64 {
                        return err(format!(
                            "ota_upload out of order: got {offset}, want {}",
                            s.end()
                        ));
                    }
                    if s.bytes.len() + bytes.len() <= OTA_STAGE_CAP {
                        s.bytes.extend(bytes.iter().copied());
                        break;
                    }
                }
                if waited_ms >= OTA_STAGE_WAIT_MS {
                    return err("ota buffer stalled (CAN side not draining)".into());
                }
                std::thread::sleep(Duration::from_millis(5));
                waited_ms += 5;
            }
            let committed = state.ota.progress.lock().sent;
            ok(serde_json::json!({ "committed": committed }))
        }

        Command::OtaFlash { node, size, crc } => {
            *state.ota.staged.lock() = Staged::default();
            *state.ota.progress.lock() = OtaProgress {
                active: true,
                sent: 0,
                total: size,
                result: None,
            };
            if node == sdm::Node::Logger as u8 {
                // Self-update: open our own OTA slot (the erase inside
                // `esp_ota_begin` blocks here for a few seconds — the client's
                // read timeout must allow for it), then drain `staged` into it
                // on a worker thread as chunks arrive.
                let ota: &'static mut EspOta = match EspOta::new() {
                    Ok(o) => Box::leak(Box::new(o)),
                    Err(e) => {
                        return err(format!(
                            "self-update unavailable (no OTA partition table, \
                             or one was already attempted — reboot to retry): {e}"
                        ))
                    }
                };
                let update: EspOtaUpdate<'static> =
                    match ota.initiate_update_with_known_size(size as usize) {
                        Ok(u) => u,
                        Err(e) => return err(format!("esp_ota_begin failed: {e}")),
                    };
                let state = Arc::clone(state);
                if std::thread::Builder::new()
                    .stack_size(8192)
                    .spawn(move || self_flash(state, update, size, crc))
                    .is_err()
                {
                    return err("could not spawn self-flash thread".into());
                }
            } else {
                *state.ota.request.lock() = Some(OtaRequest { node, size, crc });
            }
            ok(serde_json::Value::Null)
        }

        Command::OtaStatus => {
            let p = *state.ota.progress.lock();
            ok(serde_json::json!({
                "active": p.active,
                "sent": p.sent,
                "total": p.total,
                "result": p.result,
            }))
        }
    }
}

struct EspFlash(Option<EspOtaUpdate<'static>>);

impl sdm::ota::Flash for EspFlash {
    fn begin(&mut self, _size: u32) -> Result<(), sdm::ota::FlashError> {
        Ok(())
    }
    fn write(&mut self, _offset: u32, data: &[u8]) -> Result<(), sdm::ota::FlashError> {
        self.0
            .as_mut()
            .ok_or(sdm::ota::FlashError)?
            .write(data)
            .map_err(|_| sdm::ota::FlashError)
    }
    fn end(&mut self) -> Result<(), sdm::ota::FlashError> {
        self.0
            .take()
            .ok_or(sdm::ota::FlashError)?
            .complete()
            .map_err(|_| sdm::ota::FlashError)
    }
}

/// Worker for a logger self-update: drains firmware bytes out of `state.ota.staged`
fn self_flash(state: Arc<State>, update: EspOtaUpdate<'static>, size: u32, crc: u32) {
    let fail = |code: u8| {
        let mut p = state.ota.progress.lock();
        p.active = false;
        p.result = Some(code);
    };

    let mut ota = sdm::ota::Ota::new(EspFlash(Some(update)));
    if let Err(e) = ota.begin(size, crc) {
        return fail(e.0 as u8);
    }

    let mut idle_ms = 0u32;
    while ota.progress() < size {
        let chunk: Vec<u8> = {
            let mut s = state.ota.staged.lock();
            let v: Vec<u8> = s.bytes.drain(..).collect();
            s.base += v.len() as u32;
            v
        };
        if chunk.is_empty() {
            if idle_ms >= OTA_SELF_IDLE_TIMEOUT_MS {
                return fail(OTA_SELF_STARVED); // drops `ota` -> esp_ota_abort
            }
            std::thread::sleep(Duration::from_millis(10));
            idle_ms += 10;
            continue;
        }
        idle_ms = 0;
        let off = ota.progress();
        if let Err(e) = ota.chunk(off, &chunk) {
            log::error!("self-OTA chunk failed at {off}: {:?}", e);
            return fail(e.0 as u8);
        }
        state.ota.progress.lock().sent = ota.progress();
    }

    match ota.end() {
        Ok(()) => {
            {
                let mut p = state.ota.progress.lock();
                p.active = false;
                p.result = Some(0);
            }
            log::info!("self-OTA verified; rebooting into the new image");
            std::thread::sleep(Duration::from_millis(OTA_SELF_REBOOT_DELAY_MS));
            unsafe { esp_restart() };
        }
        Err(e) => {
            log::error!("self-OTA verify failed: {:?}", e);
            fail(e.0 as u8);
        }
    }
}

fn hex_decode(s: &str) -> Result<Vec<u8>, String> {
    if s.len() % 2 != 0 {
        return Err("odd-length hex".into());
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).map_err(|e| format!("bad hex: {e}")))
        .collect()
}

fn reader_thread(
    mut driver: UsbHsCdc,
    cmd_tx: mpsc::SyncSender<String>,
    resp_rx: mpsc::Receiver<String>,
) {
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
