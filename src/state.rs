use crate::gnss::Fix;
use crate::imu::ImuReading;
use crate::resources::ResourceMonitor;
use crate::status::DeviceStatus;
use parking_lot::Mutex;
use std::collections::HashMap;

#[derive(Default)]
pub struct State {
    pub sensors: Sensors,
    pub status: DeviceStatus,
    pub resources: Mutex<ResourceMonitor>,
}

#[derive(Default)]
pub struct Sensors {
    pub can: Mutex<HashMap<String, Vec<u8>>>,
    pub adc: Mutex<HashMap<u8, u16>>,
    pub gps: Mutex<Option<Fix>>,
    pub imu: Mutex<Option<ImuReading>>,
}
