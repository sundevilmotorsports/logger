//! Thin wrapper over UART0: the link to the desktop client ([`crate::serial`]
//! speaks the protocol on top). This board wires UART0 straight to a CP2102N
//! USB-UART bridge (the `ESP_PROG` connector), the same port `idf.py flash`
//! uses -- there's no native USB peripheral wired on this hardware (the S3's
//! USB D+/D- pins are reused here for the GNSS UART), and the bridge chip's
//! RTS/DTR lines already drive the ROM bootloader's auto-reset circuit in
//! hardware, so unlike the USB-CDC link this replaces, no firmware-side
//! reset-pulse detection is needed to make `espflash`/`idf.py` flashing work.

use esp_idf_svc::hal::delay::NON_BLOCK;
use esp_idf_svc::hal::uart::UartDriver;
use esp_idf_svc::sys::{EspError, ESP_ERR_TIMEOUT};

pub struct UartSerial(UartDriver<'static>);

impl UartSerial {
    pub fn new(driver: UartDriver<'static>) -> Self {
        Self(driver)
    }

    /// Non-blocking: returns whatever is already buffered, 0 if nothing is available
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, EspError> {
        match self.0.read(buf, NON_BLOCK) {
            Err(e) if e.code() == ESP_ERR_TIMEOUT => Ok(0),
            other => other,
        }
    }

    pub fn write(&mut self, bytes: &[u8]) -> Result<usize, EspError> {
        self.0.write(bytes)
    }
}
