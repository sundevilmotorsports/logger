mod adc;
mod bootloader;
mod can;
mod configuration;
mod gnss;
mod imu;
mod logging;
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
use imu::Imu;
use log::info;
use serial::serial_task;
use state::SensorState;
use static_cell::StaticCell;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use usb_hs::UsbHsCdc;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    Configuration::init().expect("config init failed");
    info!("Configuration initalized");

    let p = Peripherals::take().expect("failed to take peripherals");
    let state = Arc::new(SensorState::default());
    info!("Peripherials");

    // Real SD hardware is dead
    state.status.sd.store(true, Ordering::Relaxed);

    let spi = init_spi_bus(p.spi2, p.pins.gpio30, p.pins.gpio29, p.pins.gpio31);
    info!("SPI initalized");

    let can = init_can(spi.clone(), p.pins.gpio34, p.pins.gpio11);
    if can.is_some() {
        state.status.can.store(true, Ordering::Relaxed);
        info!("can initialized");
    }

    let adc = init_adc(spi.clone(), p.pins.gpio27); // TODO: Check spi
    if adc.is_some() {
        state.status.adc.store(true, Ordering::Relaxed);
        info!("adc initialized");
    }

    let imu = init_imu(p.i2c0, p.pins.gpio2, p.pins.gpio3);
    if imu.is_some() {
        state.status.imu.store(true, Ordering::Relaxed);
        info!("imu initialized");
    }

    if init_gnss(p.uart1, p.pins.gpio33, p.pins.gpio32, state.clone()) {
        state.status.gnss.store(true, Ordering::Relaxed);
        info!("gnss initialized");
    }

    let usb_hs = UsbHsCdc::new().expect("USB HS CDC init failed");
    state.status.usb_hs.store(true, Ordering::Relaxed);
    info!("usb hs initialized");

    serial::spawn_reader(usb_hs);
    info!("serial communication initialized");

    static EXECUTOR: StaticCell<embassy_executor::Executor> = StaticCell::new();
    let executor = EXECUTOR.init(embassy_executor::Executor::new());

    let _log_timer = logging::log_timer(state.clone()).expect("log_task");
    state.status.logging.store(true, Ordering::Relaxed);
    info!("logging initialized, spawning tasks");

    executor.run(move |spawner| {
        spawner.spawn(serial_task(state.clone()).expect("serial_task"));
        if let Some(adc) = adc {
            adc.spawn(spawner, state.clone());
        }
        if let Some(imu) = imu {
            imu.spawn(spawner, state.clone());
        }
        spawner.spawn(bootloader::watch_task().expect("bootloader_watch_task"));
        if let Some(can) = can {
            can.spawn(spawner, state);
        }
    })
}

fn init_spi_bus(
    spi2: impl SpiAnyPins + 'static,
    sck: impl OutputPin + 'static,
    mosi: impl OutputPin + 'static,
    miso: impl InputPin + 'static,
) -> Arc<SpiDriver<'static>> {
    let spi = SpiDriver::new(spi2, sck, mosi, Some(miso), &SpiDriverConfig::new())
        .expect("SPI driver init failed");
    Arc::new(spi)
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
    state: Arc<SensorState>,
) -> bool {
    let Ok(gnss_uart) = UartDriver::new(
        uart1,
        tx,
        rx,
        Option::<esp_idf_svc::hal::gpio::AnyIOPin>::None,
        Option::<esp_idf_svc::hal::gpio::AnyIOPin>::None,
        &uart_config::Config::new().baudrate(Hertz(38_400)),
    )
    .inspect_err(|e| log::error!("GNSS UART init failed: {e:?}")) else {
        return false;
    };
    gnss::spawn_reader(gnss_uart, state);
    true
}
