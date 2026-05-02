use std::ops::{Deref, DerefMut};
use std::sync::{LazyLock, Mutex};
use serde::{Deserialize, Serialize};
use crate::can::CanDevice;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Configuration {
    pub loaded: bool,
    pub can_devices: Vec<CanDevice>,
}

pub static CONFIGURATION: LazyLock<Mutex<Configuration>> = LazyLock::new(|| {
    Mutex::new(Configuration { loaded: false, can_devices: vec![] })
});

impl Configuration {
    pub fn load_json(json: &str) -> Result<(), serde_json::Error> {
        let config: Configuration = serde_json::from_str(json)?;

        let mut config_guard = CONFIGURATION.lock().unwrap();
        config_guard.can_devices = config.can_devices;
        config_guard.loaded = true;

        Ok(())
    }

    pub fn json() -> Result<String, serde_json::Error> {
        serde_json::to_string(CONFIGURATION.lock().unwrap().deref())
    }
}
