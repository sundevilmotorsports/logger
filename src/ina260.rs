//! INA260 bus voltage/current monitor on I2C. Polls the raw current and bus
//! voltage registers on a timer and stores the latest in `State::sensors::power`.
//! Values are stored raw (matching the C++ logger's convention of not scaling
//! on-device): current is 1.25 mA/LSB signed, bus voltage is 1.25 mV/LSB unsigned.

use crate::state::State;
use esp_idf_svc::hal::delay;
use esp_idf_svc::hal::i2c::I2cDriver;
use esp_idf_svc::sys::EspError;
use serde::Serialize;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

const ADDRESS: u8 = 0x40;
const REG_CURRENT: u8 = 0x01;
const REG_BUS_VOLTAGE: u8 = 0x02;

#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct PowerReading {
    /// Raw INA260 current register, 1.25 mA/LSB, signed.
    pub current_raw: i16,
    /// Raw INA260 bus voltage register, 1.25 mV/LSB, unsigned.
    pub voltage_raw: u16,
}

pub struct Ina260 {
    i2c: I2cDriver<'static>,
}

impl Ina260 {
    pub fn new(i2c: I2cDriver<'static>) -> Self {
        Self { i2c }
    }

    fn read_register(&mut self, reg: u8) -> Result<u16, EspError> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(ADDRESS, &[reg], &mut buf, delay::BLOCK)?;
        Ok(u16::from_be_bytes(buf))
    }

    pub fn read(&mut self) -> Result<PowerReading, EspError> {
        Ok(PowerReading {
            current_raw: self.read_register(REG_CURRENT)? as i16,
            voltage_raw: self.read_register(REG_BUS_VOLTAGE)?,
        })
    }

    pub fn spawn(self, state: Arc<State>) -> bool {
        std::thread::Builder::new()
            .stack_size(4096)
            .spawn(move || run(self, state))
            .inspect_err(|e| log::error!("ina260 thread spawn failed: {e:?}"))
            .is_ok()
    }
}

fn run(mut power: Ina260, state: Arc<State>) -> ! {
    crate::supervisor::run(move || -> Result<(), EspError> {
        state.status.power.store(false, Ordering::Relaxed);
        // Cheap presence check before declaring the sensor healthy.
        power.read()?;
        log::info!("ina260 initialized");

        loop {
            let reading = power.read()?;
            *state.sensors.power.lock() = Some(reading);
            state.status.power.store(true, Ordering::Relaxed);
            std::thread::sleep(Duration::from_millis(50));
        }
    })
}
