mod sd;

use esp_idf_svc::hal::gpio::{AnyInputPin, AnyOutputPin};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::rmt::ClockSource::Default;
use esp_idf_svc::hal::sd::SdCardConfiguration;
use log::info;

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Hello, world!");

    let peripherals = Peripherals::take()?;

    // TODO: use correct pins
    let sd = sd::SdCard::new(
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

    Ok(())
}
