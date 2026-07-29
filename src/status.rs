use std::sync::atomic::{AtomicBool, Ordering};

#[derive(Default)]
pub struct DeviceStatus {
    pub adc: AtomicBool,
    pub can: AtomicBool,
    pub gnss: AtomicBool,
    pub imu: AtomicBool,
    pub logging: AtomicBool,
    pub sd: AtomicBool,
    pub usb_hs: AtomicBool,
}

impl DeviceStatus {
    pub fn to_json(&self) -> serde_json::Value {
        let load = |flag: &AtomicBool| flag.load(Ordering::Relaxed);
        serde_json::json!({
            "adc": load(&self.adc),
            "can": load(&self.can),
            "gnss": load(&self.gnss),
            "imu": load(&self.imu),
            "logging": load(&self.logging),
            "sd": load(&self.sd),
            "usb_hs": load(&self.usb_hs),
        })
    }
}
