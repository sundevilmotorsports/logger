use crate::can::CanDevice;
use esp_idf_svc::nvs::{EspDefaultNvs, EspDefaultNvsPartition, EspNvs};
use serde::{Deserialize, Serialize};
use std::fs;
use std::sync::{LazyLock, Mutex};

#[derive(Serialize, Deserialize)]
pub struct Configuration {
    pub can_devices: Vec<CanDevice>,
    #[serde(skip)]
    nvs: Option<EspDefaultNvs>,
}

pub static CONFIGURATION: LazyLock<Mutex<Configuration>> = LazyLock::new(|| {
    Mutex::new(Configuration {
        can_devices: vec![],
        nvs: None,
    })
});

impl Configuration {
    /// Initialize NVS and load `config.json` from the SD card.
    pub fn init() -> anyhow::Result<()> {
        let nvs_partition = EspDefaultNvsPartition::take()?;
        let nvs = EspNvs::new(nvs_partition, "logger", true)?;

        match fs::read_to_string("/sdcard/config.json") {
            Ok(json) => {
                if let Err(e) = Self::load_json(&json) {
                    log::warn!("Failed to parse config.json: {e}");
                }
            }
            Err(e) => log::warn!("config.json not found on SD card: {e}"),
        }

        CONFIGURATION.lock().unwrap().nvs = Some(nvs);
        Ok(())
    }

    pub fn nvs(&self) -> Option<&EspDefaultNvs> {
        self.nvs.as_ref()
    }

    pub fn nvs_mut(&mut self) -> Option<&mut EspDefaultNvs> {
        self.nvs.as_mut()
    }

    pub fn load_json(json: &str) -> Result<(), serde_json::Error> {
        let config: Configuration = serde_json::from_str(json)?;
        let mut guard = CONFIGURATION.lock().unwrap();
        guard.can_devices = config.can_devices;
        Ok(())
    }

    pub fn json() -> Result<String, serde_json::Error> {
        serde_json::to_string(&*CONFIGURATION.lock().unwrap())
    }
}
