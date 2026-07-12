use crate::{can, gnss};
use crate::configuration::{Configuration, CONFIGURATION};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use esp_idf_svc::hal::delay;
use esp_idf_svc::hal::usb_serial::UsbSerialDriver;
use esp_idf_svc::sys::esp_timer_get_time;
use esp_idf_svc::systime::EspSystemTime;
use serde::Deserialize;

const MAX_PAYLOAD: usize = 16 * 1024;

pub static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, String, 4> = Channel::new();
pub static RESPONSE_CHANNEL: Channel<CriticalSectionRawMutex, String, 4> = Channel::new();

#[derive(Deserialize)]
#[serde(tag = "cmd", rename_all = "snake_case")]
enum Command {
    Ping,
    Status,
    GetConfig,
    SetConfig { args: Configuration },
    Frames,
    Uptime,
    Gps,
}

#[embassy_executor::task]
pub async fn serial_task() {
    loop {
        let payload = COMMAND_CHANNEL.receive().await;
        let response = handle_command(&payload);
        RESPONSE_CHANNEL.send(response).await;
    }
}

fn handle_command(payload: &str) -> String {
    let ok =
        |data: serde_json::Value| serde_json::json!({"ok": true, "data": data}).to_string() + "\n";
    let err = |msg: String| serde_json::json!({"ok": false, "error": msg}).to_string() + "\n";

    let cmd: Command = match serde_json::from_str(payload) {
        Ok(c) => c,
        Err(e) => return err(format!("invalid command: {e}")),
    };

    match cmd {
        Command::Ping => ok(serde_json::json!("pong")),

        Command::Status => {
            let loaded = !CONFIGURATION.lock().unwrap().can_devices.is_empty();
            ok(serde_json::json!({ "config_loaded": loaded }))
        }

        Command::GetConfig => match Configuration::json() {
            Ok(j) => {
                let v: serde_json::Value = serde_json::from_str(&j).unwrap_or_default();
                ok(v)
            }
            Err(e) => err(e.to_string()),
        },

        Command::SetConfig { args } => {
            let mut guard = CONFIGURATION.lock().unwrap();
            guard.can_devices = args.can_devices;
            drop(guard);
            ok(serde_json::Value::Null)
        }

        Command::Frames => {
            let snapshot: Vec<_> = {
                let frames = can::LATEST_FRAMES.lock().unwrap();
                frames
                    .iter()
                    .map(|(id, frame)| {
                        let signals: Vec<_> = frame
                            .signals
                            .iter()
                            .map(|s| serde_json::json!({"name": s.name, "bytes": s.bytes}))
                            .collect();
                        serde_json::json!({
                            "id": format!("0x{id:03x}"),
                            "fd": frame.fd,
                            "signals": signals,
                        })
                    })
                    .collect()
            };
            ok(serde_json::json!(snapshot))
        }

        Command::Uptime => {
            let time = unsafe { esp_timer_get_time() as f64 / 1_000_000.0 } as u64;
            ok(serde_json::json!({ "uptime_seconds": time }))
        }

        Command::Gps => match &*gnss::LATEST_FIX.lock().unwrap() {
            Some(fix) => ok(serde_json::to_value(fix).unwrap_or_default()),
            None => err("no fix".into()),
        },
    }
}

pub fn spawn_reader(driver: UsbSerialDriver<'static>) {
    std::thread::Builder::new()
        .stack_size(8192)
        .spawn(move || reader_thread(driver))
        .expect("serial reader thread");
}

fn reader_thread(mut driver: UsbSerialDriver<'static>) {
    let mut buf = Vec::<u8>::new();
    let mut tmp = [0u8; 64];

    loop {
        while let Ok(resp) = RESPONSE_CHANNEL.try_receive() {
            let _ = driver.write(resp.as_bytes(), delay::BLOCK);
        }

        let n = driver.read(&mut tmp, 10).unwrap_or(0);
        for &b in &tmp[..n] {
            if b == b'\n' {
                let payload = String::from_utf8_lossy(&buf).trim().to_string();
                buf.clear();
                if !payload.is_empty() {
                    if COMMAND_CHANNEL.try_send(payload).is_err() {
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
