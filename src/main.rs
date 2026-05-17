mod can;
mod configuration;
mod sd;
mod serial;

use crate::configuration::{Configuration, CONFIGURATION};
use crate::serial::{COMMAND_CHANNEL, RESPONSE_CHANNEL};
use can::CanBus;
use embassy_time::Timer;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::gpio::{AnyInputPin, Input, PinDriver, Pull};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::sd::SdCardConfiguration;
use esp_idf_svc::hal::spi::config::Config as SpiConfig;
use esp_idf_svc::hal::spi::{SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use esp_idf_svc::hal::usb_serial::{UsbSerialConfig, UsbSerialDriver};
use log::info;
use sd::SdCard;
use static_cell::StaticCell;
use std::sync::{Arc, Mutex};

type CanBusType = CanBus<SpiDeviceDriver<'static, SpiDriver<'static>>>;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().expect("failed to take peripherals");

    // TODO: use correct pins
    let _sd = SdCard::new(
        peripherals.sdmmc0,
        peripherals.pins.gpio4,
        peripherals.pins.gpio5,
        peripherals.pins.gpio6,
        peripherals.pins.gpio7,
        peripherals.pins.gpio8,
        peripherals.pins.gpio9,
        AnyInputPin::none(),
        AnyInputPin::none(),
        &SdCardConfiguration::default(),
    )
    .expect("SD card init failed");

    Configuration::init().expect("config init failed");

    // TODO: use correct pins
    let spi = SpiDriver::new(
        peripherals.spi2,
        peripherals.pins.gpio10,       // SCK
        peripherals.pins.gpio11,       // MOSI
        Some(peripherals.pins.gpio12), // MISO
        &SpiDriverConfig::new(),
    )
    .expect("SPI driver init failed");
    let spi_device = SpiDeviceDriver::new(spi, Some(peripherals.pins.gpio13), &SpiConfig::new())
        .expect("SPI device init failed");

    let mut delay = FreeRtos;
    let mut bus = CanBus::new(spi_device, &mut delay).expect("CAN init failed");

    for device in &*CONFIGURATION.lock().unwrap().can_devices {
        bus.register_can_device(device.clone());
    }

    let int_pin = PinDriver::input(peripherals.pins.gpio14, Pull::Up) // TODO: correct INT pin
        .expect("INT pin init failed");

    let serial = UsbSerialDriver::new(
        peripherals.usb_serial,
        peripherals.pins.gpio19, // D-
        peripherals.pins.gpio20, // D+
        &UsbSerialConfig::default(),
    )
    .expect("USB serial init failed");
    serial::spawn_reader(serial);

    let bus = Arc::new(Mutex::new(bus));

    static EXECUTOR: StaticCell<embassy_executor::Executor> = StaticCell::new();
    let executor = EXECUTOR.init(embassy_executor::Executor::new());
    executor.run(|spawner| {
        spawner.spawn(can_poll_task(int_pin, Arc::clone(&bus)).expect("can_poll_task"));
        spawner.spawn(log_task(Arc::clone(&bus)).expect("log_task"));
        spawner.spawn(serial_task(Arc::clone(&bus)).expect("serial_task"));
    });
}

#[embassy_executor::task]
async fn can_poll_task(mut int_pin: PinDriver<'static, Input>, bus: Arc<Mutex<CanBusType>>) {
    loop {
        int_pin.wait_for_falling_edge().await.ok();

        if let Err(e) = bus.lock().unwrap().poll_once() {
            log::warn!("CAN poll error: {:?}", e);
        }
    }
}

#[embassy_executor::task]
async fn log_task(bus: Arc<Mutex<CanBusType>>) {
    loop {
        {
            let guard = bus.lock().unwrap();
            for (id, frame) in guard.all_frames() {
                info!("0x{:03x} [fd={}]", id, frame.fd);
                for sig in &frame.signals {
                    info!("  {} = {:?}", sig.name, sig.bytes);
                }
            }
        }

        Timer::after_millis(50).await;
    }
}

#[embassy_executor::task]
async fn serial_task(bus: Arc<Mutex<CanBusType>>) {
    loop {
        let line = COMMAND_CHANNEL.receive().await;
        let response = handle_command(&line, &bus);
        RESPONSE_CHANNEL.send(response).await;
    }
}

fn handle_command(line: &str, bus: &Arc<Mutex<CanBusType>>) -> String {
    let ok = |data: serde_json::Value| serde_json::json!({"ok": true, "data": data}).to_string() + "\n";
    let err = |msg: String| serde_json::json!({"ok": false, "error": msg}).to_string() + "\n";

    let msg: serde_json::Value = match serde_json::from_str(line) {
        Ok(v) => v,
        Err(e) => return err(format!("invalid JSON: {e}")),
    };

    let cmd = match msg["cmd"].as_str() {
        Some(c) => c,
        None => return err("missing 'cmd' field".into()),
    };

    match cmd {
        "ping" => ok(serde_json::json!("pong")),

        "status" => {
            let loaded = !CONFIGURATION.lock().unwrap().can_devices.is_empty();
            ok(serde_json::json!({ "config_loaded": loaded }))
        }

        "get_config" => match Configuration::json() {
            Ok(j) => {
                let v: serde_json::Value = serde_json::from_str(&j).unwrap_or_default();
                ok(v)
            }
            Err(e) => err(e.to_string()),
        },

        "set_config" => {
            let args = msg["args"].to_string();
            match Configuration::load_json(&args) {
                Ok(()) => ok(serde_json::Value::Null),
                Err(e) => err(e.to_string()),
            }
        }

        "frames" => {
            let snapshot: Vec<_> = {
                let guard = bus.lock().unwrap();
                guard
                    .all_frames()
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

        _ => err(format!("unknown command '{cmd}'")),
    }
}
