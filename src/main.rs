mod adc;
mod bootloader;
mod can;
mod configuration;
mod gnss;
mod logging;
mod sd;
mod sd_fake;
mod serial;
mod usb_hs;

use can::{can_poll_task, CanBusType};
use configuration::Configuration;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::gpio::{AnyIOPin, AnyInputPin, PinDriver, Pull};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::sd::SdCardConfiguration;
use esp_idf_svc::hal::spi::config::Config as SpiConfig;
use esp_idf_svc::hal::spi::{SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use esp_idf_svc::hal::uart::{config as uart_config, UartDriver};
use esp_idf_svc::hal::units::Hertz;
use log::info;
use sd::SdCard;
use serial::serial_task;
use static_cell::StaticCell;
use std::sync::{Arc, Mutex};
use usb_hs::UsbHsCdc;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().expect("failed to take peripherals");

    // Real SD hardware is dead (CMD/CLK nets open on the board) -> sd_fake::SD
    // (an in-memory fake) stands in until it's fixed. Swap back to the block
    // below then.
    // let _sd = SdCard::new(
    //     peripherals.sdmmc0,
    //     peripherals.pins.gpio44, // cmd
    //     peripherals.pins.gpio43, // clk
    //     peripherals.pins.gpio39, // d0
    //     peripherals.pins.gpio40, // d1
    //     peripherals.pins.gpio41, // d2
    //     peripherals.pins.gpio42, // d3
    //     AnyInputPin::none(),
    //     AnyInputPin::none(),
    //     &SdCardConfiguration::default(),
    // )
    // .expect("SD card init failed");

    Configuration::init().expect("config init failed");

    // TODO: use correct pins. SPI1 isn't usable for general peripherals on
    // this chip, so CAN and ADC share this one SPI2 bus via separate CS pins.
    let spi = SpiDriver::new(
        peripherals.spi2,
        peripherals.pins.gpio30,       // SCK
        peripherals.pins.gpio29,       // MOSI
        Some(peripherals.pins.gpio31), // MISO
        &SpiDriverConfig::new(),
    )
    .expect("SPI driver init failed");
    let spi = Arc::new(spi);
    let spi_device =
        SpiDeviceDriver::new(spi.clone(), Some(peripherals.pins.gpio34), &SpiConfig::new())
            .expect("SPI device init failed");
    
    let mut delay = FreeRtos;
    let mut bus = can::CanBus::new(spi_device, &mut delay).expect("CAN init failed");
    match bus.self_test(&mut delay) {
        Ok(()) => info!("CAN self-test passed"),
        Err(e) => log::error!("CAN self-test failed: {:?}", e),
    }
    //
    // for device in &*CONFIGURATION.lock().unwrap().can_devices {
    //     bus.register_can_device(device.clone());
    // }
    //
    // let int_pin = PinDriver::input(peripherals.pins.gpio14, Pull::Up) // TODO: correct INT pin
    //     .expect("INT pin init failed");

    let gnss_uart = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio33, // TX
        peripherals.pins.gpio32, // RX
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &uart_config::Config::new().baudrate(Hertz(38_400)),
    )
    .expect("GNSS UART init failed");
    gnss::spawn_reader(gnss_uart);
    
    let usb_hs = UsbHsCdc::new().expect("USB HS CDC init failed");
    serial::spawn_reader(usb_hs);

    // ADC is a TI ADS7951 (12-bit, 8ch) on the shared SPI2 bus, own CS pin.
    let adc_spi_device = // TODO: correct CS pin
        SpiDeviceDriver::new(spi.clone(), Some(peripherals.pins.gpio25), &SpiConfig::new())
            .expect("ADC SPI device init failed");
    let adc_bus: Arc<Mutex<adc::AdcBusType>> = Arc::new(Mutex::new(adc::AdcBus::new(
        adc_spi_device,
        adc::Range::R5V,
    )));

    // let bus: Arc<Mutex<CanBusType>> = Arc::new(Mutex::new(bus));

    static EXECUTOR: StaticCell<embassy_executor::Executor> = StaticCell::new();
    let executor = EXECUTOR.init(embassy_executor::Executor::new());
    executor.run(move |spawner| {
        // spawner.spawn(can_poll_task(int_pin, bus).expect("can_poll_task"));
        spawner.spawn(serial_task().expect("serial_task"));
        spawner.spawn(adc::adc_poll_task(adc_bus).expect("adc_poll_task"));
        spawner.spawn(logging::log_task().expect("log_task"));
    });
}
