use crate::adc::{AdcChannel, LATEST_ADC};
use crate::can::{Signals, LATEST_FRAMES};
use crate::configuration::CONFIGURATION;
use crate::gnss::LATEST_FIX;
use crate::sd_fake::SD;
use embassy_time::Timer;
use std::collections::HashMap;
use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, Ordering};

/// Whether `log_task` is currently writing rows. Toggled remotely via the
/// `set_logging` serial command; the timer keeps ticking either way so a
/// resume takes effect on the next 50ms tick.
pub static ACTIVE: AtomicBool = AtomicBool::new(true);

fn signal_columns() -> Vec<String> {
    CONFIGURATION
        .lock()
        .unwrap()
        .can_devices
        .iter()
        .flat_map(|dev| -> Vec<String> {
            match &dev.signals {
                Signals::Fixed(sigs) => sigs.iter().map(|s| s.name.clone()).collect(),
                Signals::Muxed { groups, .. } => groups
                    .iter()
                    .flat_map(|g| g.signals.iter().map(|s| s.name.clone()))
                    .collect(),
            }
        })
        .collect()
}

fn configured_adc_channels() -> Vec<AdcChannel> {
    CONFIGURATION.lock().unwrap().adc_channels.clone()
}

fn write_hex(mut sink: impl Write, bytes: &[u8]) -> io::Result<()> {
    for b in bytes {
        write!(sink, "{b:02x}")?;
    }
    Ok(())
}

fn write_row(mut sink: impl Write, columns: &[String], adc_channels: &[AdcChannel]) -> io::Result<()> {
    write!(sink, "{}", embassy_time::Instant::now().as_millis())?;

    let frames = LATEST_FRAMES.lock().unwrap();
    let signals: HashMap<&str, &[u8]> = frames
        .values()
        .flat_map(|f| &f.signals)
        .map(|s| (s.name.as_str(), s.bytes.as_slice()))
        .collect();
    for name in columns {
        write!(sink, ",")?;
        if let Some(bytes) = signals.get(name.as_str()) {
            write_hex(&mut sink, bytes)?;
        }
    }
    drop(frames);

    match &*LATEST_FIX.lock().unwrap() {
        Some(fix) => write!(
            sink,
            ",{},{},{},{},{}",
            fix.lat, fix.lon, fix.alt_m, fix.sats, fix.quality
        )?,
        None => write!(sink, ",,,,,")?, // lat, lon, alt, sats, quality
    }

    let adc = LATEST_ADC.lock().unwrap();
    for ch in adc_channels {
        write!(sink, ",")?;
        if let Some(&raw) = adc.get(&ch.channel) {
            write!(sink, "{}", ch.apply(raw))?;
        }
    }
    drop(adc);

    writeln!(sink)
}

#[embassy_executor::task]
pub async fn log_task() {
    let columns = signal_columns();
    let adc_channels = configured_adc_channels();
    let adc_names: Vec<&str> = adc_channels.iter().map(|c| c.name.as_str()).collect();
    if let Err(e) = writeln!(
        SD.lock().unwrap(),
        "timestamp,{},lat,lon,alt,sats,quality,{}",
        columns.join(","),
        adc_names.join(","),
    ) {
        log::error!("failed to write log header: {e}");
    }

    loop {
        if ACTIVE.load(Ordering::Relaxed) {
            if let Err(e) = write_row(&mut *SD.lock().unwrap(), &columns, &adc_channels) {
                log::warn!("failed to write log row: {e}");
            }
        }
        Timer::after_millis(50).await;
    }
}
