use embassy_time::Timer;
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use esp_idf_svc::hal::delay::Ets;
use esp_idf_svc::hal::i2c::I2cDriver;
use serde::Serialize;
use std::sync::{LazyLock, Mutex};

pub type ImuBusType = ImuBus<I2cDriver<'static>>;

const ADDRESS: u8 = 0x6b;
const REG_CTRL1_XL: u8 = 0x10;
const REG_CTRL2_G: u8 = 0x11;
const REG_FUNC_CFG_ACCESS: u8 = 0x01;
const REG_SENSOR_HUB_1: u8 = 0x02;
const REG_MASTER_CONFIG: u8 = 0x14;
const REG_SLV0_ADD: u8 = 0x15;
const REG_SLV0_SUBADD: u8 = 0x16;
const REG_SLV0_CONFIG: u8 = 0x17;
const REG_DATAWRITE_SLV0: u8 = 0x21;
const REG_STATUS_MASTER: u8 = 0x22;
const REG_OUT_TEMP_L: u8 = 0x20;
const REG_OUTX_L_G: u8 = 0x22;
const REG_OUTX_L_A: u8 = 0x28;

/// FUNC_CFG_ACCESS.reg_access values (bits 7:6) that switch which bank
/// addresses 0x02-0x22 refer to.
const BANK_USER: u8 = 0x00;
const BANK_SENSOR_HUB: u8 = 0x01 << 6;

/// ODR field (bits 7:4 of CTRL1_XL/CTRL2_G): 0100 = 104 Hz.
const ODR_104HZ: u8 = 0b0100 << 4;

/// Sensitivity at the chip's power-on-reset full-scale defaults (+-2 g,
/// +-250 dps)
const ACCEL_SENSITIVITY_MG: f32 = 0.061;
const GYRO_SENSITIVITY_MDPS: f32 = 8.750;

/// IIS2MDC magnetometer, reachable only via the ISM330DHCX's sensor hub
/// (aux I2C master), not directly from the main bus.
const MAG_ADDRESS: u8 = 0x1e;
const MAG_REG_CFG_REG_A: u8 = 0x60;
const MAG_REG_OUTX_L: u8 = 0x68;
/// COMP_TEMP_EN=1, ODR=100Hz (0b11), MD=continuous (0b00).
const MAG_CFG_CONTINUOUS_100HZ: u8 = 0b1_0_0_0_11_00;
/// 1.5 mGauss/LSB, converted to microtesla
const MAG_SENSITIVITY_UT: f32 = 0.15;

#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct ImuReading {
    pub accel_g: [f32; 3],
    pub gyro_dps: [f32; 3],
    pub temp_c: f32,
    pub mag_ut: [f32; 3],
}

pub static LATEST_IMU: LazyLock<Mutex<Option<ImuReading>>> = LazyLock::new(|| Mutex::new(None));

pub struct ImuBus<I2C> {
    i2c: I2C,
}

impl<I2C: I2c> ImuBus<I2C> {
    pub fn new(mut i2c: I2C) -> Result<Self, I2C::Error> {
        i2c.write(ADDRESS, &[REG_CTRL1_XL, ODR_104HZ])?;
        i2c.write(ADDRESS, &[REG_CTRL2_G, ODR_104HZ])?;

        let mut bus = Self { i2c };
        bus.setup_magnetometer()?;
        Ok(bus)
    }

    fn shub_write(&mut self, reg: u8, val: u8) -> Result<(), I2C::Error> {
        self.i2c.write(ADDRESS, &[REG_FUNC_CFG_ACCESS, BANK_SENSOR_HUB])?;
        self.i2c.write(ADDRESS, &[reg, val])?;
        self.i2c.write(ADDRESS, &[REG_FUNC_CFG_ACCESS, BANK_USER])
    }

    /// One-time write to put the magnetometer in continuous mode, then
    /// reconfigure the hub to continuously read its output register instead.
    fn setup_magnetometer(&mut self) -> Result<(), I2C::Error> {
        // Clear any stale sensor hub state before configuring it
        self.shub_write(REG_MASTER_CONFIG, 0b1000_0000)?; // rst_master_regs=1
        self.shub_write(REG_MASTER_CONFIG, 0b0000_0000)?; // rst_master_regs=0

        // Point SLV0 at a write to the magnetometer's CFG_REG_A.
        self.shub_write(REG_SLV0_ADD, MAG_ADDRESS << 1 /* rw=write */)?;
        self.shub_write(REG_SLV0_SUBADD, MAG_REG_CFG_REG_A)?;
        self.shub_write(REG_DATAWRITE_SLV0, MAG_CFG_CONTINUOUS_100HZ)?;
        // master_on + shub_pu_en + write_once, triggered by the next XL sample.
        self.shub_write(REG_MASTER_CONFIG, 0b0100_1100)?;

        let mut done = false;
        for _ in 0..20 {
            Ets.delay_ms(10);
            self.i2c
                .write(ADDRESS, &[REG_FUNC_CFG_ACCESS, BANK_SENSOR_HUB])?;
            let mut status = [0u8];
            self.i2c
                .write_read(ADDRESS, &[REG_STATUS_MASTER], &mut status)?;
            self.i2c.write(ADDRESS, &[REG_FUNC_CFG_ACCESS, BANK_USER])?;
            if status[0] & 0x80 != 0 {
                done = true;
                break;
            }
        }
        if !done {
            log::warn!("IMU: magnetometer sensor-hub write never completed, mag data may be stale");
        }

        // Reconfigure SLV0 for a continuous 6-byte read of the mag's output.
        self.shub_write(REG_SLV0_ADD, (MAG_ADDRESS << 1) | 1 /* rw=read */)?;
        self.shub_write(REG_SLV0_SUBADD, MAG_REG_OUTX_L)?;
        self.shub_write(REG_SLV0_CONFIG, 6 /* slave0_numop */)?;
        self.shub_write(REG_MASTER_CONFIG, 0b0000_1100)
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

        self.i2c
            .write(ADDRESS, &[REG_FUNC_CFG_ACCESS, BANK_SENSOR_HUB])?;
        let mut mag = [0u8; 6];
        let mag_result = self
            .i2c
            .write_read(ADDRESS, &[REG_SENSOR_HUB_1], &mut mag);
        self.i2c.write(ADDRESS, &[REG_FUNC_CFG_ACCESS, BANK_USER])?;
        mag_result?;
        let mag_ut = axes(&mag).map(|raw| raw as f32 * MAG_SENSITIVITY_UT);

        Ok(ImuReading {
            accel_g,
            gyro_dps,
            temp_c,
            mag_ut,
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
