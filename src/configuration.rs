//! The logger's runtime config: which CAN devices and ADC channels to decode.
//! Held in the [`CONFIGURATION`] global, read from [`CONFIG_PATH`] on the SD
//! card at boot, written back there on every update, and pushed from the
//! desktop client over [`serial`](crate::serial).

use crate::adc::AdcChannel;
use crate::can::CanDevice;
use parking_lot::Mutex;
use serde::{Deserialize, Serialize};
use std::fs;
use std::sync::LazyLock;

pub const CONFIG_PATH: &str = "/sdcard/config.json";

#[derive(Serialize, Deserialize)]
pub struct Configuration {
    pub can_devices: Vec<CanDevice>,
    #[serde(default)]
    pub adc_channels: Vec<AdcChannel>,
}

pub static CONFIGURATION: LazyLock<Mutex<Configuration>> = LazyLock::new(|| {
    Mutex::new(Configuration {
        can_devices: vec![],
        adc_channels: vec![],
    })
});

impl Configuration {
    /// Load the config from [`CONFIG_PATH`] on the SD card
    pub fn init() {
        match fs::read_to_string(CONFIG_PATH) {
            Ok(json) => {
                if let Err(e) = Self::load_json(&json) {
                    log::warn!("Failed to parse {CONFIG_PATH}: {e}");
                }
            }
            Err(e) => log::warn!("{CONFIG_PATH} not found on SD card: {e}"),
        }
    }

    pub fn load_json(json: &str) -> Result<(), serde_json::Error> {
        let config: Configuration = serde_json::from_str(json)?;
        let mut guard = CONFIGURATION.lock();
        guard.can_devices = config.can_devices;
        guard.adc_channels = config.adc_channels;
        Ok(())
    }

    pub fn json() -> Result<String, serde_json::Error> {
        serde_json::to_string(&*CONFIGURATION.lock())
    }

    pub fn save() -> std::io::Result<()> {
        let json = Self::json().map_err(std::io::Error::other)?;
        fs::write(CONFIG_PATH, json)
    }
}
