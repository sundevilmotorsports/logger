//! The logger's runtime config: which CAN devices and ADC channels to decode.
//! Held in the [`CONFIGURATION`] global, persisted as JSON in NVS, and pushed
//! from the desktop client over [`serial`](crate::serial).

use crate::adc::AdcChannel;
use crate::can::CanDevice;
use esp_idf_svc::nvs::{EspDefaultNvs, EspDefaultNvsPartition, EspNvs};
use parking_lot::Mutex;
use serde::{Deserialize, Serialize};
use std::fs;
use std::sync::LazyLock;

#[derive(Serialize, Deserialize)]
pub struct Configuration {
    pub can_devices: Vec<CanDevice>,
    #[serde(default)]
    pub adc_channels: Vec<AdcChannel>,
    #[serde(skip)]
    nvs: Option<EspDefaultNvs>,
}

pub static CONFIGURATION: LazyLock<Mutex<Configuration>> = LazyLock::new(|| {
    Mutex::new(Configuration {
        can_devices: vec![],
        adc_channels: vec![],
        nvs: None,
    })
});

impl Configuration {
    /// Initialize NVS and load `config.json` from the SD card.
    pub fn init() {
        match EspDefaultNvsPartition::take().and_then(|p| EspNvs::new(p, "logger", true)) {
            Ok(nvs) => CONFIGURATION.lock().nvs = Some(nvs),
            Err(e) => log::warn!("NVS init failed, continuing without persistent storage: {e}"),
        }

        match fs::read_to_string("/sdcard/config.json") {
            Ok(json) => {
                if let Err(e) = Self::load_json(&json) {
                    log::warn!("Failed to parse config.json: {e}");
                }
            }
            Err(e) => log::warn!("config.json not found on SD card: {e}"),
        }
    }

    #[allow(dead_code)]
    pub fn nvs(&self) -> Option<&EspDefaultNvs> {
        self.nvs.as_ref()
    }

    #[allow(dead_code)]
    pub fn nvs_mut(&mut self) -> Option<&mut EspDefaultNvs> {
        self.nvs.as_mut()
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
}
