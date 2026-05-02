mod can;
mod configuration;
mod sd;

use can::{CanBus, CanDevice, Signal, SignalGroup, Signals};
use configuration::Configuration;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::gpio::{AnyInputPin, InterruptType, PinDriver, Pull};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::sd::SdCardConfiguration;
use esp_idf_svc::hal::spi::config::Config as SpiConfig;
use esp_idf_svc::hal::spi::{SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use log::info;
use sd::SdCard;
use std::sync::{mpsc, Arc, Mutex};
use std::time::Duration;
use crate::configuration::CONFIGURATION;

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;

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
    )?;

    // TODO: use correct pins
    let spi = SpiDriver::new(
        peripherals.spi2,
        peripherals.pins.gpio10,       // SCK
        peripherals.pins.gpio11,       // MOSI
        Some(peripherals.pins.gpio12), // MISO
        &SpiDriverConfig::new(),
    )?;
    let spi_device = SpiDeviceDriver::new(spi, Some(peripherals.pins.gpio13), &SpiConfig::new())?;

    let mut delay = FreeRtos;
    let mut bus =
        CanBus::new(spi_device, &mut delay).map_err(|_| anyhow::anyhow!("CAN init failed"))?;
    
    // TODO: load from config file on SD card
    for device in &*CONFIGURATION.lock().unwrap().can_devices {
        bus.register_can_device(device.clone());
    }

    let bus = Arc::new(Mutex::new(bus));

    let (int_tx, int_rx) = mpsc::sync_channel::<()>(1);
    let mut int_pin = PinDriver::input(peripherals.pins.gpio14, Pull::Up)?; // TODO: correct INT pin
    int_pin.set_interrupt_type(InterruptType::NegEdge)?;
    unsafe {
        int_pin.subscribe(move || {
            int_tx.try_send(()).ok();
        })?;
    }
    int_pin.enable_interrupt()?;

    let bus_rx = Arc::clone(&bus);
    std::thread::Builder::new()
        .stack_size(8192)
        .spawn(move || {
            //let _int_pin = int_pin; // keep pin and subscription alive, might be needed
            loop {
                int_rx.recv().ok();
                bus_rx.lock().unwrap().poll_once().ok();
            }
        })?;

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

        std::thread::sleep(Duration::from_millis(100));
    }
}
