//! [`State`] is the one shared object every thread gets an `Arc` to: latest
//! sensor readings behind mutexes, the [`DeviceStatus`](crate::status)
//! health flags, the resource monitor, and the logging on/off control.

use crate::gnss::Fix;
use crate::imu::ImuReading;
use crate::resources::ResourceMonitor;
use crate::status::DeviceStatus;
use parking_lot::Mutex;
use std::collections::{HashMap, VecDeque};
use std::sync::atomic::AtomicBool;

#[derive(Default)]
pub struct State {
    pub sensors: Sensors,
    pub status: DeviceStatus,
    pub resources: Mutex<ResourceMonitor>,
    pub logging: LoggingControl,
    pub ota: Ota,
}

#[derive(Default)]
pub struct Ota {
    /// Set by serial to start a transfer, taken by the CAN thread
    pub request: Mutex<Option<OtaRequest>>,
    pub staged: Mutex<Staged>,
    pub progress: Mutex<OtaProgress>,
}

pub struct OtaRequest {
    pub node: u8,
    pub size: u32,
    pub crc: u32,
}

/// Bytes uploaded over serial but not yet confirmed flashed by the node
#[derive(Default)]
pub struct Staged {
    pub bytes: VecDeque<u8>,
    pub base: u32,
}

impl Staged {
    /// Image offset one past the last byte accepted from serial
    pub fn end(&self) -> u32 {
        self.base + self.bytes.len() as u32
    }
}

#[derive(Clone, Copy, Default)]
pub struct OtaProgress {
    pub active: bool,
    pub sent: u32,
    pub total: u32,
    /// `None` while running; `Some(0)` = success, `Some(n)` = `can_ota_result`
    pub result: Option<u8>,
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
