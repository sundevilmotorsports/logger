//! SDM26 ESP32-S3 data logger firmware.
//!
//! `main` takes the peripherals, brings up each sensor, and hands it to
//! [`supervisor::run`] on its own thread. A sensor that fails to initialize is
//! logged and skipped; the rest keep running. Threads write their latest
//! readings into the shared [`state::State`]; [`logging`] samples that state on
//! a timer and appends binary rows to the SD card, and [`serial`] serves the
//! config and log files to the desktop client over UART0 (a CP2102N
//! USB-UART bridge on this board -- see `uart_serial`).

mod adc;
mod can;
mod configuration;
mod gnss;
mod ina260;
mod logging;
mod resources;
mod sd;
mod serial;
mod state;
mod status;
mod supervisor;
mod uart_serial;

use adc::Adc;
use can::Can;
use configuration::Configuration;
use esp_idf_svc::hal::can::{
    config::{Config as CanConfig, Filter as CanFilter, Mode as CanMode, Timing as CanTiming},
    CanDriver,
};
use esp_idf_svc::hal::i2c::{config::Config as I2cConfig, I2cDriver};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::spi::{
    config::{Config as SpiConfig, MODE_3},
    SpiDeviceDriver, SpiDriver, SpiDriverConfig,
};
use esp_idf_svc::hal::uart::{config as uart_config, UartDriver};
use esp_idf_svc::hal::units::Hertz;
use gnss::Gnss;
use ina260::Ina260;
use log::info;
use sd::SdCard;
use state::State;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use uart_serial::UartSerial;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let p = Peripherals::take().expect("failed to take peripherals");
    let state = Arc::new(State::default());
    info!("Peripherials");

    // SD pins are claimed inside `SdCard` via `steal()` so it
    // can remount itself after a card error
    let sd_ok = SdCard::init()
        .inspect_err(|e| log::error!("SD card init failed: {e:?}"))
        .is_ok();
    state.status.sd.store(sd_ok, Ordering::Relaxed);
    if sd_ok {
        info!("SD card initialized");
    }

    Configuration::init();
    info!("Configuration initalized");

    // ADC128S102 on SPI3 (VSPI): CS=GPIO4, CLK=GPIO5, DOUT(MISO)=GPIO6, DIN(MOSI)=GPIO7.
    let spi3 = SpiDriver::new(
        p.spi3,
        p.pins.gpio5,       // sclk
        p.pins.gpio7,       // sdo (MOSI -> ADC DIN)
        Some(p.pins.gpio6), // sdi (MISO <- ADC DOUT)
        &SpiDriverConfig::new(),
    )
    .inspect_err(|e| log::error!("ADC SPI driver init failed: {e:?}"))
    .ok();

    let adc = spi3.and_then(|spi| {
        SpiDeviceDriver::new(
            spi,
            Some(p.pins.gpio4),
            &SpiConfig::new()
                .baudrate(Hertz(10_000_000))
                .data_mode(MODE_3),
        )
        .inspect_err(|e| log::error!("ADC SPI device init failed: {e:?}"))
        .ok()
        .map(Adc::new)
    });
    if let Some(adc) = adc {
        if !adc.spawn(state.clone()) {
            log::error!("adc thread failed to start");
        }
    }

    let can_config = CanConfig::new()
        .timing(CanTiming::B1M)
        .filter(CanFilter::extended_allow_all())
        .mode(CanMode::Normal)
        .rx_queue_len(256);
    let can = CanDriver::new(p.can, p.pins.gpio8, p.pins.gpio18, &can_config)
        .inspect_err(|e| log::error!("CAN driver init failed: {e:?}"))
        .ok()
        .map(Can::new);
    if let Some(can) = can {
        if !can.spawn(state.clone()) {
            log::error!("can thread failed to start");
        }
    }

    let i2c_config = I2cConfig::new()
        .baudrate(Hertz(100_000))
        .sda_enable_pullup(false)
        .scl_enable_pullup(false);
    let power = I2cDriver::new(p.i2c0, p.pins.gpio48, p.pins.gpio47, &i2c_config)
        .inspect_err(|e| log::error!("I2C driver init failed: {e:?}"))
        .ok()
        .map(Ina260::new);
    if let Some(power) = power {
        if !power.spawn(state.clone()) {
            log::error!("ina260 thread failed to start");
        }
    }

    // NEO-F9P GNSS on UART1: TX/RX = GPIO19/20, 38400 baud.
    let gnss = UartDriver::new(
        p.uart1,
        p.pins.gpio19,
        p.pins.gpio20,
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

    let serial_link = UartDriver::new(
        p.uart0,
        p.pins.gpio43,
        p.pins.gpio44,
        Option::<esp_idf_svc::hal::gpio::AnyIOPin>::None,
        Option::<esp_idf_svc::hal::gpio::AnyIOPin>::None,
        &uart_config::Config::new().baudrate(Hertz(115_200)),
    )
    .inspect_err(|e| log::error!("serial UART init failed: {e:?}"))
    .ok()
    .map(UartSerial::new);
    if let Some(serial_link) = serial_link {
        if serial::spawn(serial_link, state.clone()) {
            state.status.serial.store(true, Ordering::Relaxed);
            info!("serial link initialized");
        }
    }

    if logging::spawn_logger(state.clone()) {
        state.status.logging.store(true, Ordering::Relaxed);
        info!("logging initialized");
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
