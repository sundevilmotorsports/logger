use esp_idf_svc::hal::delay;
use esp_idf_svc::hal::uart::UartDriver;
use serde::Serialize;
use std::sync::{LazyLock, Mutex};

#[derive(Clone, Serialize)]
pub struct Fix {
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f64,
    pub sats: u8,
    pub quality: u8,
}

pub static LATEST_FIX: LazyLock<Mutex<Option<Fix>>> = LazyLock::new(|| Mutex::new(None));

pub fn spawn_reader(driver: UartDriver<'static>) {
    std::thread::Builder::new()
        .stack_size(4096)
        .spawn(move || reader_thread(driver))
        .expect("gnss reader thread");
}

fn reader_thread(driver: UartDriver<'static>) {
    let mut buf = Vec::<u8>::new();
    let mut tmp = [0u8; 128];

    loop {
        let n = driver.read(&mut tmp, delay::BLOCK).unwrap_or(0);
        for &b in &tmp[..n] {
            if b == b'\n' {
                if let Ok(line) = std::str::from_utf8(&buf) {
                    if let Some(fix) = parse_gga(line.trim_end_matches('\r')) {
                        *LATEST_FIX.lock().unwrap() = Some(fix);
                    }
                }
                buf.clear();
            } else if buf.len() < 128 {
                // NMEA sentences are <= 82 bytes
                buf.push(b);
            } else {
                buf.clear();
            }
        }
    }
}

// $GxGGA,time,lat,N,lon,E,quality,numSV,HDOP,alt,M,...*checksum
fn parse_gga(line: &str) -> Option<Fix> {
    let (body, sum) = line.strip_prefix('$')?.split_once('*')?;
    if u8::from_str_radix(sum, 16).ok()? != body.bytes().fold(0, |a, b| a ^ b) {
        return None;
    }

    let f: Vec<&str> = body.split(',').collect();
    if f.len() < 10 || !f[0].ends_with("GGA") {
        return None;
    }
    let quality: u8 = f[6].parse().ok()?;
    if quality == 0 {
        return None; // no fix yet
    }

    Some(Fix {
        lat: dm_to_deg(f[2])? * if f[3] == "S" { -1.0 } else { 1.0 },
        lon: dm_to_deg(f[4])? * if f[5] == "W" { -1.0 } else { 1.0 },
        alt_m: f[9].parse().ok()?,
        sats: f[7].parse().ok()?,
        quality,
    })
}

// (d)ddmm.mmmmm -> decimal degrees
fn dm_to_deg(s: &str) -> Option<f64> {
    let dot = s.find('.')?;
    if dot < 2 {
        return None;
    }
    let (d, m) = s.split_at(dot - 2);
    Some(d.parse::<f64>().ok()? + m.parse::<f64>().ok()? / 60.0)
}
