mod can;
mod configuration;
mod gnss;
mod sd;
mod serial;

use crate::can::LATEST_FRAMES;
use can::{can_poll_task, CanBusType};
use configuration::{Configuration, CONFIGURATION};
use embassy_time::Timer;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::gpio::{AnyIOPin, AnyInputPin, PinDriver, Pull};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::sd::SdCardConfiguration;
use esp_idf_svc::hal::spi::config::Config as SpiConfig;
use esp_idf_svc::hal::spi::{SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use esp_idf_svc::hal::uart::{config as uart_config, UartDriver};
use esp_idf_svc::hal::units::Hertz;
use esp_idf_svc::hal::usb_serial::{UsbSerialConfig, UsbSerialDriver};
use log::info;
use sd::SdCard;
use serial::serial_task;
use static_cell::StaticCell;
use std::sync::{Arc, Mutex};

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().expect("failed to take peripherals");

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

    // TODO: use correct pins
    let spi = SpiDriver::new(
        peripherals.spi2,
        peripherals.pins.gpio30,       // SCK
        peripherals.pins.gpio29,       // MOSI
        Some(peripherals.pins.gpio31), // MISO
        &SpiDriverConfig::new(),
    )
    .expect("SPI driver init failed");
    let spi_device = SpiDeviceDriver::new(spi, Some(peripherals.pins.gpio34), &SpiConfig::new())
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

    let usb_serial = UsbSerialDriver::new(
        peripherals.usb_serial,
        peripherals.pins.gpio24, // USB D-
        peripherals.pins.gpio25, // USB D+
        &UsbSerialConfig::new(),
    )
    .expect("USB serial init failed");
    serial::spawn_reader(usb_serial);

    // let bus: Arc<Mutex<CanBusType>> = Arc::new(Mutex::new(bus));

    static EXECUTOR: StaticCell<embassy_executor::Executor> = StaticCell::new();
    let executor = EXECUTOR.init(embassy_executor::Executor::new());
    executor.run(|spawner| {
        // spawner.spawn(can_poll_task(int_pin, bus).expect("can_poll_task"));
        // spawner.spawn(log_task().expect("log_task"));
        spawner.spawn(serial_task().expect("serial_task"));
    });
}

#[embassy_executor::task]
pub async fn log_task() {
    loop {
        {
            let frames = LATEST_FRAMES.lock().unwrap();
            for (id, frame) in frames.iter() {
                info!("0x{:03x} [fd={}]", id, frame.fd);
                for sig in &frame.signals {
                    info!("  {} = {:?}", sig.name, sig.bytes);
                }
            }
        }

        Timer::after_millis(50).await;
    }
}
