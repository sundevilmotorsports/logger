use crate::state::State;
use chrono::{NaiveDate, NaiveDateTime, NaiveTime};
use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::hal::uart::UartDriver;
use parking_lot::Mutex;
use serde::Serialize;
use std::sync::atomic::Ordering;
use std::sync::{Arc, LazyLock};
use std::time::{Duration, Instant};

const STALE_TIMEOUT: Duration = Duration::from_secs(5);
const READ_TIMEOUT: Duration = Duration::from_secs(1);

#[derive(Clone, Serialize)]
pub struct Fix {
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f64,
    pub sats: u8,
    pub quality: u8,
    pub utc: Option<NaiveDateTime>,
}

/// Most recent RMC date, needed to timestamp GGA's time-only field. Internal to this thread.
static LATEST_DATE: LazyLock<Mutex<Option<NaiveDate>>> = LazyLock::new(|| Mutex::new(None));

pub struct Gnss(UartDriver<'static>);

impl Gnss {
    pub fn new(driver: UartDriver<'static>) -> Self {
        Self(driver)
    }

    pub fn spawn(self, state: Arc<State>) -> bool {
        std::thread::Builder::new()
            .stack_size(4096)
            .spawn(move || poll_loop(self.0, state))
            .inspect_err(|e| log::error!("gnss thread spawn failed: {e:?}"))
            .is_ok()
    }
}

fn poll_loop(driver: UartDriver<'static>, state: Arc<State>) {
    let mut buf = Vec::<u8>::new();
    let mut tmp = [0u8; 128];
    let mut last_fix = Instant::now();
    loop {
        let n = driver
            .read(&mut tmp, TickType::from(READ_TIMEOUT).into())
            .unwrap_or(0);
        for &b in &tmp[..n] {
            if b == b'\n' {
                if let Ok(line) = std::str::from_utf8(&buf) {
                    let line = line.trim_end_matches('\r');
                    if let Some(date) = parse_rmc_date(line) {
                        *LATEST_DATE.lock() = Some(date);
                    }
                    if let Some(fix) = parse_gga(line) {
                        *state.sensors.gps.lock() = Some(fix);
                        last_fix = Instant::now();
                        state.status.gnss.store(true, Ordering::Relaxed);
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
        if last_fix.elapsed() > STALE_TIMEOUT {
            state.status.gnss.store(false, Ordering::Relaxed);
        }
    }
}

fn checksum_body(line: &str) -> Option<&str> {
    let (body, sum) = line.strip_prefix('$')?.split_once('*')?;
    (u8::from_str_radix(sum, 16).ok()? == body.bytes().fold(0, |a, b| a ^ b)).then_some(body)
}

// $GxGGA,time,lat,N,lon,E,quality,numSV,HDOP,alt,M,...*checksum
fn parse_gga(line: &str) -> Option<Fix> {
    let body = checksum_body(line)?;
    let f: Vec<&str> = body.split(',').collect();
    if f.len() < 10 || !f[0].ends_with("GGA") {
        return None;
    }

    let quality: u8 = f[6].parse().ok()?;
    if quality == 0 {
        return None; // no fix yet
    }

    let utc = parse_time(f[1])
        .and_then(|time| LATEST_DATE.lock().map(|date| NaiveDateTime::new(date, time)));

    Some(Fix {
        lat: dm_to_deg(f[2])? * if f[3] == "S" { -1.0 } else { 1.0 },
        lon: dm_to_deg(f[4])? * if f[5] == "W" { -1.0 } else { 1.0 },
        alt_m: f[9].parse().ok()?,
        sats: f[7].parse().ok()?,
        quality,
        utc,
    })
}

// $GxRMC,time,status,lat,N,lon,E,speed,track,ddmmyy,...*checksum
fn parse_rmc_date(line: &str) -> Option<NaiveDate> {
    let body = checksum_body(line)?;
    let f: Vec<&str> = body.split(',').collect();
    if f.len() < 10 || !f[0].ends_with("RMC") {
        return None;
    }
    if f[2] != "A" {
        return None; // status void, don't trust the date
    }
    let d = f[9];
    if d.len() != 6 {
        return None;
    }
    let day: u32 = d[0..2].parse().ok()?;
    let month: u32 = d[2..4].parse().ok()?;
    let yy: i32 = d[4..6].parse().ok()?;

    let year = if yy < 80 { 2000 + yy } else { 1900 + yy };
    NaiveDate::from_ymd_opt(year, month, day)
}

// hhmmss.sss -> NaiveTime
fn parse_time(s: &str) -> Option<NaiveTime> {
    if s.len() < 6 {
        return None;
    }
    let h: u32 = s[0..2].parse().ok()?;
    let m: u32 = s[2..4].parse().ok()?;
    let sec: f64 = s[4..].parse().ok()?;
    let whole_sec = sec.trunc() as u32;
    let milli = ((sec.fract()) * 1000.0).round() as u32;
    NaiveTime::from_hms_milli_opt(h, m, whole_sec, milli)
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
