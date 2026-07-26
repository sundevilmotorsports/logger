use crate::gnss::Fix;
use crate::imu::ImuReading;
use std::collections::HashMap;
use std::sync::Mutex;

#[derive(Default)]
pub struct SensorState {
    pub can_signals: Mutex<HashMap<String, Vec<u8>>>,
    pub adc: Mutex<HashMap<u8, u16>>,
    pub gps: Mutex<Option<Fix>>,
    pub imu: Mutex<Option<ImuReading>>,
}
