use esp_idf_svc::sys::{
    esp, tinyusb_cdcacm_itf_t, tinyusb_cdcacm_itf_t_TINYUSB_CDC_ACM_0, tinyusb_cdcacm_read,
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
