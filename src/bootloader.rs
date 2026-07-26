use embassy_time::Timer;
use esp_idf_svc::sys::esp_restart;
use std::sync::atomic::{AtomicBool, Ordering};

const LP_SYSTEM_REG_SYS_CTRL_REG: *mut u32 = 0x5011_0008 as *mut u32;
const FORCE_DOWNLOAD_BOOT: u32 = 1 << 2;

/// Set by the USB line-state-changed callback (usb_hs.rs) when it sees an
/// espflash-style reset pulse.
pub static REQUESTED: AtomicBool = AtomicBool::new(false);

/// Sets the force-download-boot bit and restarts
pub fn reboot_to_bootloader() -> ! {
    unsafe {
        let current = core::ptr::read_volatile(LP_SYSTEM_REG_SYS_CTRL_REG);
        core::ptr::write_volatile(LP_SYSTEM_REG_SYS_CTRL_REG, current | FORCE_DOWNLOAD_BOOT);
        esp_restart();
    }
}

#[embassy_executor::task]
pub async fn watch_task() {
    loop {
        if REQUESTED.swap(false, Ordering::Relaxed) {
            reboot_to_bootloader();
        }
        Timer::after_millis(20).await;
    }
}
