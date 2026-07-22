use esp_idf_svc::sys::{
    cdcacm_event_t, cdcacm_event_type_t_CDC_EVENT_LINE_STATE_CHANGED, esp, esp_restart,
    tinyusb_cdcacm_itf_t, tinyusb_cdcacm_itf_t_TINYUSB_CDC_ACM_0, tinyusb_cdcacm_read,
    tinyusb_cdcacm_write_flush, tinyusb_cdcacm_write_queue, tinyusb_config_cdcacm_t,
    tinyusb_config_t, tinyusb_driver_install, tusb_cdc_acm_init, EspError, TickType_t,
};

const CDC_ACM_0: tinyusb_cdcacm_itf_t = tinyusb_cdcacm_itf_t_TINYUSB_CDC_ACM_0;

pub struct UsbHsCdc;

impl UsbHsCdc {
    pub fn new() -> Result<Self, EspError> {
        let config = tinyusb_config_t {
            external_phy: false,
            self_powered: false,
            vbus_monitor_io: -1,
            ..Default::default()
        };
        esp!(unsafe { tinyusb_driver_install(&config) })?;

        let cdc_config = tinyusb_config_cdcacm_t {
            cdc_port: CDC_ACM_0,
            callback_line_state_changed: Some(reset_on_bootloader_request),
            ..Default::default()
        };
        esp!(unsafe { tusb_cdc_acm_init(&cdc_config) })?;

        Ok(Self)
    }

    /// Non-blocking: returns whatever is already buffered, 0 if nothing is available.
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, EspError> {
        let mut n: usize = 0;
        esp!(unsafe { tinyusb_cdcacm_read(CDC_ACM_0, buf.as_mut_ptr(), buf.len(), &mut n) })?;
        Ok(n)
    }

    pub fn write(&mut self, bytes: &[u8], timeout_ticks: TickType_t) -> Result<usize, EspError> {
        let n = unsafe { tinyusb_cdcacm_write_queue(CDC_ACM_0, bytes.as_ptr(), bytes.len()) };
        esp!(unsafe { tinyusb_cdcacm_write_flush(CDC_ACM_0, timeout_ticks) })?;
        Ok(n)
    }
}

/// Reboots on espflash's DTR+RTS-low bootloader-reset signal, since this
/// port has no hardware reset circuit. Unverified on real hardware.
extern "C" fn reset_on_bootloader_request(_itf: core::ffi::c_int, event: *mut cdcacm_event_t) {
    let event = unsafe { &*event };
    if event.type_ != cdcacm_event_type_t_CDC_EVENT_LINE_STATE_CHANGED {
        return;
    }
    let state = unsafe { event.__bindgen_anon_1.line_state_changed_data };
    if !state.dtr && !state.rts {
        unsafe { esp_restart() };
    }
}
