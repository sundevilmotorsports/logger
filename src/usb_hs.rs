use crate::bootloader;
use esp_idf_svc::sys::{
    cdcacm_event_t, cdcacm_event_type_t_CDC_EVENT_LINE_STATE_CHANGED, esp, tinyusb_cdcacm_itf_t,
    tinyusb_cdcacm_itf_t_TINYUSB_CDC_ACM_0, tinyusb_cdcacm_read, tinyusb_cdcacm_write_flush,
    tinyusb_cdcacm_write_queue, tinyusb_config_cdcacm_t, tinyusb_config_t, tinyusb_driver_install,
    tusb_cdc_acm_init, EspError, TickType_t,
};
use std::sync::atomic::{AtomicBool, Ordering};

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

    /// `tinyusb_cdcacm_write_queue` only queues as many bytes as fit in the
    /// TX FIFO (512 by default)
    pub fn write(&mut self, bytes: &[u8], timeout_ticks: TickType_t) -> Result<usize, EspError> {
        let mut sent = 0;
        while sent < bytes.len() {
            let n = unsafe {
                tinyusb_cdcacm_write_queue(CDC_ACM_0, bytes[sent..].as_ptr(), bytes.len() - sent)
            };
            esp!(unsafe { tinyusb_cdcacm_write_flush(CDC_ACM_0, timeout_ticks) })?;
            if n == 0 {
                break; // host isn't draining; avoid spinning forever
            }
            sent += n;
        }
        Ok(sent)
    }
}

static SAW_RESET_PULSE: AtomicBool = AtomicBool::new(false);

/// Might work
extern "C" fn reset_on_bootloader_request(_itf: core::ffi::c_int, event: *mut cdcacm_event_t) {
    let event = unsafe { &*event };
    if event.type_ != cdcacm_event_type_t_CDC_EVENT_LINE_STATE_CHANGED {
        return;
    }
    let state = unsafe { event.__bindgen_anon_1.line_state_changed_data };

    if state.dtr && state.rts {
        SAW_RESET_PULSE.store(true, Ordering::Relaxed);
    } else if !state.dtr && !state.rts && SAW_RESET_PULSE.swap(false, Ordering::Relaxed) {
        bootloader::reboot_to_bootloader();
    }
}
