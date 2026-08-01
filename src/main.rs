mod adc;
mod bootloader;
mod can;
mod configuration;
mod gnss;
mod imu;
mod logging;
mod resources;
mod sd;
mod sd_fake;
mod serial;
mod state;
mod status;
mod usb_hs;

use adc::Adc;
use can::Can;
use configuration::Configuration;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::gpio::{InputPin, OutputPin, PinDriver, Pull};
use esp_idf_svc::hal::i2c::{config::Config as I2cConfig, I2c as I2cPeripheral, I2cDriver};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::spi::{
    config::Config as SpiConfig, SpiAnyPins, SpiDeviceDriver, SpiDriver, SpiDriverConfig,
};
use esp_idf_svc::hal::uart::{config as uart_config, Uart as UartPeripheral, UartDriver};
use esp_idf_svc::hal::units::Hertz;
use gnss::Gnss;
use imu::Imu;
use log::info;
use state::State;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use usb_hs::UsbHsCdc;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    Configuration::init();
    info!("Configuration initalized");

    let p = Peripherals::take().expect("failed to take peripherals");
    let state = Arc::new(State::default());
    info!("Peripherials");

    // Real SD hardware is dead
    state.status.sd.store(true, Ordering::Relaxed);

    let spi = init_spi_bus(p.spi2, p.pins.gpio30, p.pins.gpio29, p.pins.gpio31);
    if spi.is_some() {
        info!("SPI initalized");
    }

    if let Some(can) = spi
        .clone()
        .and_then(|spi| init_can(spi, p.pins.gpio34, p.pins.gpio11))
    {
        if can.spawn(state.clone()) {
            state.status.can.store(true, Ordering::Relaxed);
            info!("can initialized");
        }
    }

    if let Some(adc) = spi.and_then(|spi| init_adc(spi, p.pins.gpio27)) {
        if adc.spawn(state.clone()) {
            state.status.adc.store(true, Ordering::Relaxed);
            info!("adc initialized");
        }
    }

    if let Some(imu) = init_imu(p.i2c0, p.pins.gpio2, p.pins.gpio3) {
        if imu.spawn(state.clone()) {
            state.status.imu.store(true, Ordering::Relaxed);
            info!("imu initialized");
        }
    }

    if let Some(gnss) = init_gnss(p.uart1, p.pins.gpio33, p.pins.gpio32) {
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

    // Everything runs on its own thread now; park the main thread forever.
    loop {
        std::thread::park();
    }
}

fn init_spi_bus(
    spi2: impl SpiAnyPins + 'static,
    sck: impl OutputPin + 'static,
    mosi: impl OutputPin + 'static,
    miso: impl InputPin + 'static,
) -> Option<Arc<SpiDriver<'static>>> {
    SpiDriver::new(spi2, sck, mosi, Some(miso), &SpiDriverConfig::new())
        .inspect_err(|e| log::error!("SPI driver init failed: {e:?}"))
        .ok()
        .map(Arc::new)
}

/// Initializes the MCP2518FD and runs its internal-loopback self-test.
/// Any failure just leaves CAN unavailable rather than taking the whole
/// device down -- ADC/IMU/GNSS/logging can all still work without it.
fn init_can(
    spi: Arc<SpiDriver<'static>>,
    cs: impl OutputPin + 'static,
    int: impl InputPin + 'static,
) -> Option<Can> {
    let spi_device = SpiDeviceDriver::new(spi, Some(cs), &SpiConfig::new())
        .inspect_err(|e| log::error!("CAN SPI device init failed: {e:?}"))
        .ok()?;
    let int_pin = PinDriver::input(int, Pull::Up)
        .inspect_err(|e| log::error!("CAN INT pin init failed: {e:?}"))
        .ok()?;

    let mut delay = FreeRtos;
    let mut can = Can::new(spi_device, int_pin, &mut delay)
        .inspect_err(|e| log::error!("CAN init failed: {e:?}"))
        .ok()?;
    match can.self_test(&mut delay) {
        Ok(()) => info!("CAN self-test passed"),
        Err(e) => log::error!("CAN self-test failed: {:?}", e),
    }

    Some(can)
}

fn init_adc(spi: Arc<SpiDriver<'static>>, cs: impl OutputPin + 'static) -> Option<Adc> {
    let spi_device = SpiDeviceDriver::new(spi, Some(cs), &SpiConfig::new())
        .inspect_err(|e| log::error!("ADC SPI device init failed: {e:?}"))
        .ok()?;
    Some(Adc::new(spi_device, adc::Range::R5V))
}

fn init_imu(
    i2c0: impl I2cPeripheral + 'static,
    sda: impl InputPin + OutputPin + 'static,
    scl: impl InputPin + OutputPin + 'static,
) -> Option<Imu> {
    let i2c = I2cDriver::new(i2c0, sda, scl, &I2cConfig::new())
        .inspect_err(|e| log::error!("I2C driver init failed: {e:?}"))
        .ok()?;
    Imu::new(i2c)
        .inspect_err(|e| log::error!("IMU init failed: {e:?}"))
        .ok()
}

fn init_gnss(
    uart1: impl UartPeripheral + 'static,
    tx: impl OutputPin + 'static,
    rx: impl InputPin + 'static,
) -> Option<Gnss> {
    let uart = UartDriver::new(
        uart1,
        tx,
        rx,
        Option::<esp_idf_svc::hal::gpio::AnyIOPin>::None,
        Option::<esp_idf_svc::hal::gpio::AnyIOPin>::None,
        &uart_config::Config::new().baudrate(Hertz(38_400)),
    )
    .inspect_err(|e| log::error!("GNSS UART init failed: {e:?}"))
    .ok()?;
    Some(Gnss::new(uart))
}
