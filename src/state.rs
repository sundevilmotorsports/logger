use crate::gnss::Fix;
use crate::imu::ImuReading;
use crate::resources::ResourceMonitor;
use crate::status::DeviceStatus;
use parking_lot::Mutex;
use std::collections::HashMap;
use std::sync::atomic::AtomicBool;

#[derive(Default)]
pub struct State {
    pub sensors: Sensors,
    pub status: DeviceStatus,
    pub resources: Mutex<ResourceMonitor>,
    pub logging: LoggingControl,
}

#[derive(Default)]
pub struct Sensors {
    pub can: Mutex<HashMap<String, Vec<u8>>>,
    pub adc: Mutex<HashMap<u8, u16>>,
    pub gps: Mutex<Option<Fix>>,
    pub imu: Mutex<Option<ImuReading>>,
}

pub struct LoggingControl {
    pub active: AtomicBool,
    pub config_changed: AtomicBool,
}

impl Default for LoggingControl {
    fn default() -> Self {
        Self {
            active: AtomicBool::new(true),
            config_changed: AtomicBool::new(false),
        }
    }
}
