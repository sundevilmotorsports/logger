use crate::state::State;
use embedded_hal::delay::DelayNs;
use esp_idf_svc::hal::delay::{self, Ets};
use esp_idf_svc::hal::i2c::I2cDriver;
use esp_idf_svc::sys::EspError;
use serde::Serialize;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

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

/// FUNC_CFG_ACCESS.reg_access bits 7:6 select which bank addresses 0x02-0x22 refer to.
const BANK_USER: u8 = 0x00;
const BANK_SENSOR_HUB: u8 = 0x01 << 6;

/// ODR field (bits 7:4 of CTRL1_XL/CTRL2_G): 0100 = 104 Hz.
const ODR_104HZ: u8 = 0b0100 << 4;

/// Sensitivity at the chip's power-on-reset full-scale defaults
const ACCEL_SENSITIVITY_MG: f32 = 0.061;
const GYRO_SENSITIVITY_MDPS: f32 = 8.750;

/// bit4 pass_through_mode, bit3 shub_pu_en, bit2 master_on, bits1:0 aux_sens_on).
const MASTER_CONFIG_RESET: u8 = 0b1000_0000;
const MASTER_CONFIG_IDLE: u8 = 0b0000_0000;
/// master_on + shub_pu_en + write_once, triggered by the next accel sample.
const MASTER_CONFIG_WRITE_ONCE: u8 = 0b0100_1100;
/// Same as above but write_once cleared -- a standing continuous read.
const MASTER_CONFIG_CONTINUOUS_READ: u8 = 0b0000_1100;
/// STATUS_MASTER bit7: the one-shot write completed.
const STATUS_WR_ONCE_DONE: u8 = 0x80;

/// IIS2MDC magnetometer, reachable only via the ISM330DHCX's sensor-hub (aux I2C master).
const MAG_ADDRESS: u8 = 0x1e;
const MAG_REG_CFG_REG_A: u8 = 0x60;
const MAG_REG_OUTX_L: u8 = 0x68;
/// COMP_TEMP_EN=1, ODR=100Hz (0b11), MD=continuous (0b00).
const MAG_CFG_CONTINUOUS_100HZ: u8 = 0b1_0_0_0_11_00;
/// 1.5 mGauss/LSB, converted to microtesla (1 mGauss = 0.1 uT).
const MAG_SENSITIVITY_UT: f32 = 0.15;

#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct ImuReading {
    pub accel_g: [f32; 3],
    pub gyro_dps: [f32; 3],
    pub temp_c: f32,
    pub mag_ut: [f32; 3],
}

pub struct Imu {
    i2c: I2cDriver<'static>,
}

impl Imu {
    pub fn new(i2c: I2cDriver<'static>) -> Self {
        Self { i2c }
    }

    pub fn init(&mut self) -> Result<(), EspError> {
        self.i2c
            .write(ADDRESS, &[REG_CTRL1_XL, ODR_104HZ], delay::BLOCK)?;
        self.i2c
            .write(ADDRESS, &[REG_CTRL2_G, ODR_104HZ], delay::BLOCK)?;
        self.setup_magnetometer()
    }

    pub fn read(&mut self) -> Result<ImuReading, EspError> {
        Ok(ImuReading {
            accel_g: self.read_axes(REG_OUTX_L_A, ACCEL_SENSITIVITY_MG / 1000.0)?,
            gyro_dps: self.read_axes(REG_OUTX_L_G, GYRO_SENSITIVITY_MDPS / 1000.0)?,
            temp_c: self.read_temperature()?,
            mag_ut: self.read_magnetometer()?,
        })
    }

    fn read_temperature(&mut self) -> Result<f32, EspError> {
        let mut temp = [0u8; 2];
        self.i2c
            .write_read(ADDRESS, &[REG_OUT_TEMP_L], &mut temp, delay::BLOCK)?;
        Ok((i16::from_le_bytes(temp) as f32 / 256.0) + 25.0)
    }

    fn read_axes(&mut self, reg: u8, sensitivity: f32) -> Result<[f32; 3], EspError> {
        let mut buf = [0u8; 6];
        self.i2c
            .write_read(ADDRESS, &[reg], &mut buf, delay::BLOCK)?;
        Ok(axes(&buf).map(|raw| raw as f32 * sensitivity))
    }

    fn read_magnetometer(&mut self) -> Result<[f32; 3], EspError> {
        let buf = self.with_shub_bank(|i2c| {
            let mut buf = [0u8; 6];
            i2c.write_read(ADDRESS, &[REG_SENSOR_HUB_1], &mut buf, delay::BLOCK)?;
            Ok(buf)
        })?;
        Ok(axes(&buf).map(|raw| raw as f32 * MAG_SENSITIVITY_UT))
    }

    /// Runs `f` inside the sensor-hub register bank, then always switches back to the user bank.
    fn with_shub_bank<T>(
        &mut self,
        f: impl FnOnce(&mut I2cDriver<'static>) -> Result<T, EspError>,
    ) -> Result<T, EspError> {
        self.i2c.write(
            ADDRESS,
            &[REG_FUNC_CFG_ACCESS, BANK_SENSOR_HUB],
            delay::BLOCK,
        )?;
        let result = f(&mut self.i2c);
        self.i2c
            .write(ADDRESS, &[REG_FUNC_CFG_ACCESS, BANK_USER], delay::BLOCK)?;
        result
    }

    fn shub_write(&mut self, reg: u8, val: u8) -> Result<(), EspError> {
        self.with_shub_bank(|i2c| i2c.write(ADDRESS, &[reg, val], delay::BLOCK))
    }

    fn setup_magnetometer(&mut self) -> Result<(), EspError> {
        self.reset_sensor_hub()?;
        self.write_magnetometer_config()?;
        self.wait_for_config_write();
        self.enable_continuous_magnetometer_read()
    }

    fn reset_sensor_hub(&mut self) -> Result<(), EspError> {
        self.shub_write(REG_MASTER_CONFIG, MASTER_CONFIG_RESET)?;
        self.shub_write(REG_MASTER_CONFIG, MASTER_CONFIG_IDLE)
    }

    /// Points SLV0 at a one-time write to the magnetometer's CFG_REG_A
    fn write_magnetometer_config(&mut self) -> Result<(), EspError> {
        self.shub_write(REG_SLV0_ADD, MAG_ADDRESS << 1 /* rw=write */)?;
        self.shub_write(REG_SLV0_SUBADD, MAG_REG_CFG_REG_A)?;
        self.shub_write(REG_DATAWRITE_SLV0, MAG_CFG_CONTINUOUS_100HZ)?;
        self.shub_write(REG_MASTER_CONFIG, MASTER_CONFIG_WRITE_ONCE)
    }

    /// Polls STATUS_MASTER for the write-complete bit
    fn wait_for_config_write(&mut self) {
        for _ in 0..20 {
            Ets.delay_ms(10);
            let done = self
                .with_shub_bank(|i2c| {
                    let mut status = [0u8];
                    i2c.write_read(ADDRESS, &[REG_STATUS_MASTER], &mut status, delay::BLOCK)?;
                    Ok(status[0] & STATUS_WR_ONCE_DONE != 0)
                })
                .unwrap_or(false);
            if done {
                return;
            }
        }
        log::warn!("IMU: magnetometer sensor-hub write never completed, mag data may be stale");
    }

    fn enable_continuous_magnetometer_read(&mut self) -> Result<(), EspError> {
        self.shub_write(REG_SLV0_ADD, (MAG_ADDRESS << 1) | 1 /* rw=read */)?;
        self.shub_write(REG_SLV0_SUBADD, MAG_REG_OUTX_L)?;
        self.shub_write(REG_SLV0_CONFIG, 6 /* slave0_numop */)?;
        self.shub_write(REG_MASTER_CONFIG, MASTER_CONFIG_CONTINUOUS_READ)
    }

    pub fn spawn(self, state: Arc<State>) -> bool {
        std::thread::Builder::new()
            .stack_size(8192)
            .spawn(move || poll_loop(self, state))
            .inspect_err(|e| log::error!("imu thread spawn failed: {e:?}"))
            .is_ok()
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

const INIT_RETRY_INTERVAL: Duration = Duration::from_secs(3);

fn poll_loop(mut imu: Imu, state: Arc<State>) {
    while let Err(e) = imu.init() {
        log::error!("IMU init failed, retrying: {e:?}");
        std::thread::sleep(INIT_RETRY_INTERVAL);
    }
    log::info!("imu initialized");

    loop {
        match imu.read() {
            Ok(reading) => {
                *state.sensors.imu.lock() = Some(reading);
                state.status.imu.store(true, Ordering::Relaxed);
            }
            Err(e) => {
                log::warn!("IMU read error: {e:?}");
                state.status.imu.store(false, Ordering::Relaxed);
            }
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}
