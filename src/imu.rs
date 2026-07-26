use embassy_time::Timer;
use embedded_hal::i2c::I2c;
use esp_idf_svc::hal::i2c::I2cDriver;
use serde::Serialize;
use std::sync::{LazyLock, Mutex};

pub type ImuBusType = ImuBus<I2cDriver<'static>>;

const ADDRESS: u8 = 0x6b;
const REG_CTRL1_XL: u8 = 0x10;
const REG_CTRL2_G: u8 = 0x11;
const REG_OUT_TEMP_L: u8 = 0x20;
const REG_OUTX_L_G: u8 = 0x22;
const REG_OUTX_L_A: u8 = 0x28;

/// ODR field (bits 7:4 of CTRL1_XL/CTRL2_G): 0100 = 104 Hz.
const ODR_104HZ: u8 = 0b0100 << 4;

/// Sensitivity at the chip's power-on-reset full-scale defaults (+-2 g,
/// +-250 dps) -- FS_XL/FS_G are left untouched, only the ODR is set.
const ACCEL_SENSITIVITY_MG: f32 = 0.061;
const GYRO_SENSITIVITY_MDPS: f32 = 8.750;

#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct ImuReading {
    pub accel_g: [f32; 3],
    pub gyro_dps: [f32; 3],
    pub temp_c: f32,
}

pub static LATEST_IMU: LazyLock<Mutex<Option<ImuReading>>> = LazyLock::new(|| Mutex::new(None));

pub struct ImuBus<I2C> {
    i2c: I2C,
}

impl<I2C: I2c> ImuBus<I2C> {
    pub fn new(mut i2c: I2C) -> Result<Self, I2C::Error> {
        i2c.write(ADDRESS, &[REG_CTRL1_XL, ODR_104HZ])?;
        i2c.write(ADDRESS, &[REG_CTRL2_G, ODR_104HZ])?;
        Ok(Self { i2c })
    }

    pub fn read(&mut self) -> Result<ImuReading, I2C::Error> {
        let mut temp = [0u8; 2];
        self.i2c.write_read(ADDRESS, &[REG_OUT_TEMP_L], &mut temp)?;
        let temp_c = (i16::from_le_bytes(temp) as f32 / 256.0) + 25.0;

        let mut gyro = [0u8; 6];
        self.i2c.write_read(ADDRESS, &[REG_OUTX_L_G], &mut gyro)?;
        let gyro_dps = axes(&gyro).map(|raw| raw as f32 * GYRO_SENSITIVITY_MDPS / 1000.0);

        let mut accel = [0u8; 6];
        self.i2c.write_read(ADDRESS, &[REG_OUTX_L_A], &mut accel)?;
        let accel_g = axes(&accel).map(|raw| raw as f32 * ACCEL_SENSITIVITY_MG / 1000.0);

        Ok(ImuReading {
            accel_g,
            gyro_dps,
            temp_c,
        })
    }
}

/// Three little-endian i16 axes packed as 6 bytes (X, Y, Z).
fn axes(bytes: &[u8; 6]) -> [i16; 3] {
    [
        i16::from_le_bytes([bytes[0], bytes[1]]),
        i16::from_le_bytes([bytes[2], bytes[3]]),
        i16::from_le_bytes([bytes[4], bytes[5]]),
    ]
}

#[embassy_executor::task]
pub async fn imu_poll_task(mut bus: ImuBusType) {
    loop {
        match bus.read() {
            Ok(reading) => *LATEST_IMU.lock().unwrap() = Some(reading),
            Err(e) => log::warn!("IMU read error: {e:?}"),
        }
        Timer::after_millis(50).await;
    }
}
