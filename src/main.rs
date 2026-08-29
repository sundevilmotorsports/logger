//! ESP32-P4 CAN FD data logger firmware.
//!
//! `main` takes the peripherals, brings up each sensor, and hands it to
//! [`supervisor::run`] on its own thread. A sensor that fails to initialize is
//! logged and skipped; the rest keep running. Threads write their latest
//! readings into the shared [`state::State`]; [`logging`] samples that state on
//! a timer and appends binary rows to the SD card, and [`serial`] serves the
//! config and log files to the desktop client over USB CDC.

mod adc;
mod bootloader;
mod can;
mod configuration;
mod gnss;
mod imu;
mod logging;
mod resources;
mod sd;
mod serial;
mod state;
mod status;
mod supervisor;
mod usb_hs;

use adc::Adc;
use can::Can;
use configuration::Configuration;
use esp_idf_svc::hal::gpio::{PinDriver, Pull};
use esp_idf_svc::hal::i2c::{config::Config as I2cConfig, I2cDriver};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::sd::SdCardConfiguration;
use esp_idf_svc::hal::spi::{
    config::Config as SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig,
};
use esp_idf_svc::hal::uart::{config as uart_config, UartDriver};
use esp_idf_svc::hal::units::Hertz;
use gnss::Gnss;
use imu::Imu;
use log::info;
use sd::SdCard;
use state::State;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use usb_hs::UsbHsCdc;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let p = Peripherals::take().expect("failed to take peripherals");
    let state = Arc::new(State::default());
    info!("Peripherials");

    let sd_ok = SdCard::init(
        p.sdmmc0,
        p.pins.gpio44, // CMD
        p.pins.gpio43, // CLK
        p.pins.gpio39, // D0
        p.pins.gpio40, // D1
        p.pins.gpio41, // D2
        p.pins.gpio42, // D3
        None::<esp_idf_svc::hal::gpio::AnyIOPin>,
        None::<esp_idf_svc::hal::gpio::AnyIOPin>,
        &SdCardConfiguration::new(),
    )
    .inspect_err(|e| log::error!("SD card init failed: {e:?}"))
    .is_ok();
    state.status.sd.store(sd_ok, Ordering::Relaxed);
    if sd_ok {
        info!("SD card initialized");
    }

    Configuration::init();
    info!("Configuration initalized");

    let spi = SpiDriver::new(
        p.spi2,
        p.pins.gpio30,
        p.pins.gpio29,
        Some(p.pins.gpio31),
        &SpiDriverConfig::new(),
    )
    .inspect_err(|e| log::error!("SPI driver init failed: {e:?}"))
    .ok()
    .map(Arc::new);
    if spi.is_some() {
        info!("SPI initalized");
    }

    let can = spi.clone().and_then(|spi| {
        let spi_device = SpiDeviceDriver::new(spi, Some(p.pins.gpio34), &SpiConfig::new())
            .inspect_err(|e| log::error!("CAN SPI device init failed: {e:?}"))
            .ok()?;
        let int_pin = PinDriver::input(p.pins.gpio11, Pull::Up)
            .inspect_err(|e| log::error!("CAN INT pin init failed: {e:?}"))
            .ok()?;
        Some(Can::new(spi_device, int_pin))
    });
    if let Some(can) = can {
        if !can.spawn(state.clone()) {
            log::error!("can thread failed to start");
        }
    }

    let adc = spi.and_then(|spi| {
        SpiDeviceDriver::new(spi, Some(p.pins.gpio27), &SpiConfig::new())
            .inspect_err(|e| log::error!("ADC SPI device init failed: {e:?}"))
            .ok()
            .map(|spi_device| Adc::new(spi_device, adc::Range::R5V))
    });
    if let Some(adc) = adc {
        if !adc.spawn(state.clone()) {
            log::error!("adc thread failed to start");
        }
    }

    let imu = I2cDriver::new(p.i2c0, p.pins.gpio2, p.pins.gpio3, &I2cConfig::new())
        .inspect_err(|e| log::error!("I2C driver init failed: {e:?}"))
        .ok()
        .map(Imu::new);
    if let Some(imu) = imu {
        if !imu.spawn(state.clone()) {
            log::error!("imu thread failed to start");
        }
    }

    let gnss = UartDriver::new(
        p.uart1,
        p.pins.gpio33,
        p.pins.gpio32,
        Option::<esp_idf_svc::hal::gpio::AnyIOPin>::None,
        Option::<esp_idf_svc::hal::gpio::AnyIOPin>::None,
        &uart_config::Config::new().baudrate(Hertz(38_400)),
    )
    .inspect_err(|e| log::error!("GNSS UART init failed: {e:?}"))
    .ok()
    .map(Gnss::new);
    if let Some(gnss) = gnss {
        if gnss.spawn(state.clone()) {
            state.status.gnss.store(true, Ordering::Relaxed);
            info!("gnss initialized");
        }
    }

    let usb_hs = UsbHsCdc::new()
        .inspect_err(|e| log::error!("USB HS CDC init failed: {e:?}"))
        .ok();
    if let Some(usb_hs) = usb_hs {
        if serial::spawn(usb_hs, state.clone()) {
            state.status.usb_hs.store(true, Ordering::Relaxed);
            info!("usb hs initialized, serial communication initialized");
        }
    }

    if logging::spawn_logger(state.clone()) {
        state.status.logging.store(true, Ordering::Relaxed);
        info!("logging initialized");
    }

    if !bootloader::spawn() {
        log::error!("bootloader watch thread failed to start");
    }

    match esp_idf_svc::ota::EspOta::new().and_then(|mut ota| ota.mark_running_slot_valid()) {
        Ok(()) => info!("running image confirmed valid"),
        Err(e) => log::warn!("could not mark running image valid: {e}"),
    }

    // Everything runs on its own thread now; park the main thread forever.
    loop {
        std::thread::park();
    }
}
