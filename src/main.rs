mod can;
mod configuration;
mod sd;

use crate::configuration::{Configuration, CONFIGURATION};
use can::CanBus;
use embassy_time::Timer;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::gpio::{AnyInputPin, Input, PinDriver, Pull};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::sd::SdCardConfiguration;
use esp_idf_svc::hal::spi::config::Config as SpiConfig;
use esp_idf_svc::hal::spi::{SpiDeviceDriver, SpiDriver, SpiDriverConfig};
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

    let bus = Arc::new(Mutex::new(bus));

    static EXECUTOR: StaticCell<embassy_executor::Executor> = StaticCell::new();
    let executor = EXECUTOR.init(embassy_executor::Executor::new());
    executor.run(|spawner| {
        spawner.spawn(can_poll_task(int_pin, Arc::clone(&bus)).expect("can_poll_task"));
        spawner.spawn(log_task(bus).expect("log_task"));
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
