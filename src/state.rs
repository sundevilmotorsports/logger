//! [`State`] is the one shared object every thread gets an `Arc` to: latest
//! sensor readings behind mutexes, the [`DeviceStatus`](crate::status)
//! health flags, the resource monitor, and the logging on/off control.

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
    /// SDM nodes seen on the CAN bus (heartbeat frames), keyed by node id.
    pub can_nodes: Mutex<HashMap<u8, CanNode>>,
    pub adc: Mutex<HashMap<u8, u16>>,
    pub gps: Mutex<Option<Fix>>,
    pub imu: Mutex<Option<ImuReading>>,
}

#[derive(Clone, Copy)]
pub struct CanNode {
    /// Device type from heartbeat byte 0 (`sdm_utils::DeviceType`).
    pub device_type: u8,
    /// micros when the last heartbeat arrived.
    pub last_seen_us: i64,
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
