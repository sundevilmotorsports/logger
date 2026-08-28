//! Reboot into the ROM serial bootloader on request. [`usb_hs`](crate::usb_hs)
//! sets [`REQUESTED`] when it sees an espflash reset pulse on the CDC line; the
//! watch thread from [`spawn`] then sets the force-download-boot bit and
//! restarts, so the device can be flashed without a physical BOOT button.

use esp_idf_svc::sys::esp_restart;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

const LP_SYSTEM_REG_SYS_CTRL_REG: *mut u32 = 0x5011_0008 as *mut u32;
const FORCE_DOWNLOAD_BOOT: u32 = 1 << 2;

/// Set by usb_hs.rs's line-state callback when it sees an espflash-style reset pulse.
pub static REQUESTED: AtomicBool = AtomicBool::new(false);

/// Sets the force-download-boot bit and restarts
pub fn reboot_to_bootloader() -> ! {
    unsafe {
        let current = core::ptr::read_volatile(LP_SYSTEM_REG_SYS_CTRL_REG);
        core::ptr::write_volatile(LP_SYSTEM_REG_SYS_CTRL_REG, current | FORCE_DOWNLOAD_BOOT);
        esp_restart();
    }
}

pub fn spawn() -> bool {
    std::thread::Builder::new()
        .spawn(watch_thread)
        .inspect_err(|e| log::error!("bootloader watch thread spawn failed: {e:?}"))
        .is_ok()
}

fn watch_thread() {
    loop {
        if REQUESTED.swap(false, Ordering::Relaxed) {
            reboot_to_bootloader();
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}
