//! `esp-idf-hal` only exposes `SPI3_HOST` for the esp32/s2/s3 its
//! cfg gates were never updated for the ESP32-P4

use esp_idf_svc::hal::spi::{Spi, SpiAnyPins};
use esp_idf_svc::hal::sys::{spi_host_device_t, spi_host_device_t_SPI3_HOST};

pub struct Spi3;

impl Spi for Spi3 {
    fn device() -> spi_host_device_t {
        spi_host_device_t_SPI3_HOST
    }
}

impl SpiAnyPins for Spi3 {}
