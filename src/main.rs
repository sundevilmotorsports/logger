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
use std::sync::Arc;
use usb_hs::UsbHsCdc;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    Configuration::init().expect("config init failed");

    let p = Peripherals::take().expect("failed to take peripherals");
    let state = Arc::new(SensorState::default());

    // Real SD hardware is dead

    let spi = init_spi_bus(p.spi2, p.pins.gpio30, p.pins.gpio29, p.pins.gpio31);

    let can = init_can(spi.clone(), p.pins.gpio34, p.pins.gpio11);
    let adc = init_adc(spi.clone(), p.pins.gpio27); // TODO: Check spi

    let i2c = init_i2c_bus(p.i2c0, p.pins.gpio2, p.pins.gpio3);
    let imu = Imu::new(i2c).expect("IMU init failed");

    init_gnss(p.uart1, p.pins.gpio33, p.pins.gpio32, state.clone());

    let usb_hs = UsbHsCdc::new().expect("USB HS CDC init failed");
    serial::spawn_reader(usb_hs);

    static EXECUTOR: StaticCell<embassy_executor::Executor> = StaticCell::new();
    let executor = EXECUTOR.init(embassy_executor::Executor::new());

    let _log_timer = logging::log_timer(state.clone()).expect("log_task");

    executor.run(move |spawner| {
        spawner.spawn(serial_task(state.clone()).expect("serial_task"));
        adc.spawn(spawner, state.clone());
        imu.spawn(spawner, state.clone());
        spawner.spawn(bootloader::watch_task().expect("bootloader_watch_task"));
        can.spawn(spawner, state);
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
fn init_can(
    spi: Arc<SpiDriver<'static>>,
    cs: impl OutputPin + 'static,
    int: impl InputPin + 'static,
) -> Can {
    let spi_device =
        SpiDeviceDriver::new(spi, Some(cs), &SpiConfig::new()).expect("CAN SPI device init failed");
    
    let int_pin = PinDriver::input(int, Pull::Up).expect("CAN INT pin init failed");

    let mut delay = FreeRtos;
    let mut can = Can::new(spi_device, int_pin, &mut delay).expect("CAN init failed");
    match can.self_test(&mut delay) {
        Ok(()) => info!("CAN self-test passed"),
        Err(e) => log::error!("CAN self-test failed: {:?}", e),
    }
    
    can
}

fn init_adc(spi: Arc<SpiDriver<'static>>, cs: impl OutputPin + 'static) -> Adc {
    let spi_device =
        SpiDeviceDriver::new(spi, Some(cs), &SpiConfig::new()).expect("ADC SPI device init failed");
    Adc::new(spi_device, adc::Range::R5V)
}

fn init_i2c_bus(
    i2c0: impl I2cPeripheral + 'static,
    sda: impl InputPin + OutputPin + 'static,
    scl: impl InputPin + OutputPin + 'static,
) -> I2cDriver<'static> {
    I2cDriver::new(i2c0, sda, scl, &I2cConfig::new()).expect("I2C driver init failed")
}

fn init_gnss(
    uart1: impl UartPeripheral + 'static,
    tx: impl OutputPin + 'static,
    rx: impl InputPin + 'static,
    state: Arc<SensorState>,
) {
    let gnss_uart = UartDriver::new(
        uart1,
        tx,
        rx,
        Option::<esp_idf_svc::hal::gpio::AnyIOPin>::None,
        Option::<esp_idf_svc::hal::gpio::AnyIOPin>::None,
        &uart_config::Config::new().baudrate(Hertz(38_400)),
    )
    .expect("GNSS UART init failed");
    gnss::spawn_reader(gnss_uart, state);
}
