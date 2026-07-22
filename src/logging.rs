use crate::adc::{AdcChannel, AdcValue, LATEST_ADC};
use crate::can::{Signal, SignalValue, Signals, LATEST_SIGNALS};
use crate::configuration::CONFIGURATION;
use crate::gnss::LATEST_FIX;
use crate::sd_fake::SD;
use embassy_time::Timer;
use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, Ordering};

/// Whether `log_task` writes rows; toggled via the `set_logging` command.
pub static ACTIVE: AtomicBool = AtomicBool::new(true);

/// Header: [1] num_columns, then per column [1] name_len, [name_len] name,
/// [1] type tag (0 = 4-byte float, N>0 = N raw bytes). Rows: fixed-width,
/// back to back, in the same column order.
enum ColType {
    Float,
    Raw(u8),
}

fn configured_can_signals() -> Vec<Signal> {
    CONFIGURATION
        .lock()
        .unwrap()
        .can_devices
        .iter()
        .flat_map(|dev| -> Vec<Signal> {
            match &dev.signals {
                Signals::Fixed(sigs) => sigs.clone(),
                Signals::Muxed { groups, .. } => {
                    groups.iter().flat_map(|g| g.signals.clone()).collect()
                }
            }
        })
        .collect()
}

fn configured_adc_channels() -> Vec<AdcChannel> {
    CONFIGURATION.lock().unwrap().adc_channels.clone()
}

fn write_schema(
    mut sink: impl Write,
    can_signals: &[Signal],
    adc_channels: &[AdcChannel],
) -> io::Result<()> {
    let mut body = Vec::new();
    let mut num_cols: u8 = 0;

    let mut append = |name: &str, ty: ColType| {
        let tag = match ty {
            ColType::Float => 0u8,
            ColType::Raw(n) => n,
        };
        body.push(name.len() as u8);
        body.extend_from_slice(name.as_bytes());
        body.push(tag);
        num_cols += 1;
    };
    let col_type = |scale: Option<f32>, raw_width: u8| {
        if scale.is_some() {
            ColType::Float
        } else {
            ColType::Raw(raw_width)
        }
    };

    append("timestamp", ColType::Raw(8));
    for sig in can_signals {
        append(&sig.name, col_type(sig.scale, sig.len as u8));
    }
    for ch in adc_channels {
        append(&ch.name, col_type(ch.scale, 2));
    }
    append("lat", ColType::Float);
    append("lon", ColType::Float);
    append("alt", ColType::Float);
    append("sats", ColType::Raw(1));
    append("quality", ColType::Raw(1));

    sink.write_all(&[num_cols])?;
    sink.write_all(&body)
}

fn write_row(
    mut sink: impl Write,
    can_signals: &[Signal],
    adc_channels: &[AdcChannel],
) -> io::Result<()> {
    let ts = embassy_time::Instant::now().as_millis();
    sink.write_all(&ts.to_le_bytes())?;

    let signals = LATEST_SIGNALS.lock().unwrap();
    for sig in can_signals {
        let raw = signals.get(&sig.name).map(Vec::as_slice);
        match sig.value(raw) {
            SignalValue::Float(f) => sink.write_all(&f.to_le_bytes())?,
            SignalValue::Raw(bytes) => sink.write_all(&bytes)?,
        }
    }
    drop(signals);

    let adc = LATEST_ADC.lock().unwrap();
    for ch in adc_channels {
        let raw = adc.get(&ch.channel).copied().unwrap_or(0);
        match ch.value(raw) {
            AdcValue::Float(f) => sink.write_all(&f.to_le_bytes())?,
            AdcValue::Raw(v) => sink.write_all(&v.to_le_bytes())?,
        }
    }
    drop(adc);

    match &*LATEST_FIX.lock().unwrap() {
        Some(fix) => {
            sink.write_all(&(fix.lat as f32).to_le_bytes())?;
            sink.write_all(&(fix.lon as f32).to_le_bytes())?;
            sink.write_all(&(fix.alt_m as f32).to_le_bytes())?;
            sink.write_all(&[fix.sats])?;
            sink.write_all(&[fix.quality])?;
        }
        None => {
            sink.write_all(&0.0f32.to_le_bytes())?;
            sink.write_all(&0.0f32.to_le_bytes())?;
            sink.write_all(&0.0f32.to_le_bytes())?;
            sink.write_all(&[0u8, 0u8])?;
        }
    }

    Ok(())
}

#[embassy_executor::task]
pub async fn log_task() {
    let can_signals = configured_can_signals();
    let adc_channels = configured_adc_channels();

    // Empty so the schema is (re)written on the first tick and after every `next_log`.
    let mut current_name = String::new();

    loop {
        if ACTIVE.load(Ordering::Relaxed) {
            let mut sd = SD.lock().unwrap();
            if sd.current_name() != current_name {
                if let Err(e) = write_schema(&mut *sd, &can_signals, &adc_channels) {
                    log::error!("failed to write log schema: {e}");
                }
                current_name = sd.current_name();
            }
            if let Err(e) = write_row(&mut *sd, &can_signals, &adc_channels) {
                log::warn!("failed to write log row: {e}");
            }
        }
        Timer::after_millis(50).await;
    }
}
