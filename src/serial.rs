use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use esp_idf_svc::hal::delay;
use esp_idf_svc::hal::usb_serial::UsbSerialDriver;

const MAX_LINE: usize = 4096;

pub static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, String, 4> = Channel::new();

pub static RESPONSE_CHANNEL: Channel<CriticalSectionRawMutex, String, 4> = Channel::new();

pub fn spawn_reader(driver: UsbSerialDriver<'static>) {
    std::thread::Builder::new()
        .stack_size(8192)
        .spawn(move || reader_thread(driver))
        .expect("serial reader thread");
}

fn reader_thread(mut driver: UsbSerialDriver<'static>) {
    let mut buf = Vec::<u8>::new();
    let mut tmp = [0u8; 64];

    loop {
        while let Ok(resp) = RESPONSE_CHANNEL.try_receive() {
            let _ = driver.write(resp.as_bytes(), delay::BLOCK);
        }

        let n = driver.read(&mut tmp, 10).unwrap_or(0);
        for &b in &tmp[..n] {
            if b == b'\n' {
                let line = String::from_utf8_lossy(&buf).trim().to_string();
                buf.clear();
                if !line.is_empty() {
                    if COMMAND_CHANNEL.try_send(line).is_err() {
                        log::warn!("serial: command channel full, dropping command");
                    }
                }
            } else if b != b'\r' {
                if buf.len() < MAX_LINE {
                    buf.push(b);
                } else {
                    log::warn!("serial: line too long, discarding buffer");
                    buf.clear();
                }
            }
        }
    }
}
