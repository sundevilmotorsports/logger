use esp_idf_svc::sys::esp_restart;

const LP_SYSTEM_REG_SYS_CTRL_REG: *mut u32 = 0x5011_0008 as *mut u32;
const FORCE_DOWNLOAD_BOOT: u32 = 1 << 2;

/// Sets the force-download-boot bit and restarts
pub fn reboot_to_bootloader() -> ! {
    unsafe {
        let current = core::ptr::read_volatile(LP_SYSTEM_REG_SYS_CTRL_REG);
        core::ptr::write_volatile(LP_SYSTEM_REG_SYS_CTRL_REG, current | FORCE_DOWNLOAD_BOOT);
        esp_restart();
    }
}
