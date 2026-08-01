use crate::adc::{AdcChannel, AdcValue};
use crate::can::{Signal, SignalValue, Signals};
use crate::configuration::CONFIGURATION;
use crate::gnss::Fix;
use crate::imu::ImuReading;
use crate::sd_fake::SD;
use crate::state::State;
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::sys::esp_timer_get_time;
use std::collections::HashMap;
use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

/// Whether the logging thread writes rows; toggled via the `set_logging` command.
pub static ACTIVE: AtomicBool = AtomicBool::new(true);

/// Header: [1] num_columns, then per column [1] name_len, [name_len] name,
/// [1] type tag (0 = 4-byte float, N>0 = N raw bytes). Rows: fixed-width,
/// back to back, in the same column order.
enum ColType {
    Float,
    Raw(u8),
}

fn col_type(scale: Option<f32>, raw_width: u8) -> ColType {
    if scale.is_some() {
        ColType::Float
    } else {
        ColType::Raw(raw_width)
    }
}

trait LogSource {
    fn schema(&self, push: &mut dyn FnMut(&str, ColType));
    fn write_row(&self, sink: &mut dyn Write) -> io::Result<()>;
}

struct CanColumns<'a> {
    signals: &'a [Signal],
    latest: HashMap<String, Vec<u8>>,
}

impl LogSource for CanColumns<'_> {
    fn schema(&self, push: &mut dyn FnMut(&str, ColType)) {
        for sig in self.signals {
            push(&sig.name, col_type(sig.scale, sig.len as u8));
        }
    }

    fn write_row(&self, sink: &mut dyn Write) -> io::Result<()> {
        for sig in self.signals {
            let raw = self.latest.get(&sig.name).map(Vec::as_slice);
            match sig.value(raw) {
                SignalValue::Float(f) => sink.write_all(&f.to_le_bytes())?,
                SignalValue::Raw(bytes) => sink.write_all(&bytes)?,
            }
        }
        Ok(())
    }
}

struct AdcColumns<'a> {
    channels: &'a [AdcChannel],
    latest: HashMap<u8, u16>,
}

impl LogSource for AdcColumns<'_> {
    fn schema(&self, push: &mut dyn FnMut(&str, ColType)) {
        for ch in self.channels {
            push(&ch.name, col_type(ch.scale, 2));
        }
    }

    fn write_row(&self, sink: &mut dyn Write) -> io::Result<()> {
        for ch in self.channels {
            let raw = self.latest.get(&ch.channel).copied().unwrap_or(0);
            match ch.value(raw) {
                AdcValue::Float(f) => sink.write_all(&f.to_le_bytes())?,
                AdcValue::Raw(v) => sink.write_all(&v.to_le_bytes())?,
            }
        }
        Ok(())
    }
}

struct ImuColumns(Option<ImuReading>);

impl LogSource for ImuColumns {
    fn schema(&self, push: &mut dyn FnMut(&str, ColType)) {
        for name in [
            "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z", "imu_temp", "mag_x",
            "mag_y", "mag_z",
        ] {
            push(name, ColType::Float);
        }
    }

    fn write_row(&self, sink: &mut dyn Write) -> io::Result<()> {
        let r = self.0.unwrap_or_default();
        for v in r.accel_g {
            sink.write_all(&v.to_le_bytes())?;
        }
        for v in r.gyro_dps {
            sink.write_all(&v.to_le_bytes())?;
        }
        sink.write_all(&r.temp_c.to_le_bytes())?;
        for v in r.mag_ut {
            sink.write_all(&v.to_le_bytes())?;
        }
        Ok(())
    }
}

struct GpsColumns(Option<Fix>);

impl LogSource for GpsColumns {
    fn schema(&self, push: &mut dyn FnMut(&str, ColType)) {
        push("lat", ColType::Float);
        push("lon", ColType::Float);
        push("alt", ColType::Float);
        push("sats", ColType::Raw(1));
        push("quality", ColType::Raw(1));
    }

    fn write_row(&self, sink: &mut dyn Write) -> io::Result<()> {
        let (lat, lon, alt_m, sats, quality) = match &self.0 {
            Some(fix) => (fix.lat, fix.lon, fix.alt_m, fix.sats, fix.quality),
            None => (0.0, 0.0, 0.0, 0, 0),
        };
        sink.write_all(&(lat as f32).to_le_bytes())?;
        sink.write_all(&(lon as f32).to_le_bytes())?;
        sink.write_all(&(alt_m as f32).to_le_bytes())?;
        sink.write_all(&[sats, quality])
    }
}

fn configured_can_signals() -> Vec<Signal> {
    CONFIGURATION
        .lock()
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
    CONFIGURATION.lock().adc_channels.clone()
}

fn snapshot<'a>(
    can_signals: &'a [Signal],
    adc_channels: &'a [AdcChannel],
    state: &State,
) -> [Box<dyn LogSource + 'a>; 4] {
    [
        Box::new(CanColumns {
            signals: can_signals,
            latest: state.sensors.can.lock().clone(),
        }),
        Box::new(AdcColumns {
            channels: adc_channels,
            latest: state.sensors.adc.lock().clone(),
        }),
        Box::new(GpsColumns(state.sensors.gps.lock().clone())),
        Box::new(ImuColumns(*state.sensors.imu.lock())),
    ]
}

fn write_schema(mut sink: impl Write, sources: &[Box<dyn LogSource + '_>]) -> io::Result<()> {
    let mut body = Vec::new();
    let mut num_cols: u8 = 0;
    let mut push = |name: &str, ty: ColType| {
        let tag = match ty {
            ColType::Float => 0u8,
            ColType::Raw(n) => n,
        };
        body.push(name.len() as u8);
        body.extend_from_slice(name.as_bytes());
        body.push(tag);
        num_cols += 1;
    };

    push("timestamp", ColType::Raw(8));
    for src in sources {
        src.schema(&mut push);
    }

    sink.write_all(&[num_cols])?;
    sink.write_all(&body)
}

fn write_row(mut sink: impl Write, sources: &[Box<dyn LogSource + '_>]) -> io::Result<()> {
    let ts = (unsafe { esp_timer_get_time() } / 1_000) as u64;
    sink.write_all(&ts.to_le_bytes())?;
    for src in sources {
        src.write_row(&mut sink)?;
    }
    Ok(())
}

pub fn spawn_logger(state: Arc<State>) -> bool {
    let previous = ThreadSpawnConfiguration::get();
    if let Err(e) = (ThreadSpawnConfiguration {
        pin_to_core: Some(Core::Core1),
        ..Default::default()
    }
    .set())
    {
        log::error!("failed to configure logging thread affinity: {e:?}");
    }

    let spawned = std::thread::Builder::new()
        .stack_size(8192)
        .spawn(move || logger_thread(state))
        .inspect_err(|e| log::error!("logging thread spawn failed: {e:?}"))
        .is_ok();

    // Restore the previous config so it doesn't leak into later spawns
    if let Some(previous) = previous {
        previous.set().ok();
    }

    spawned
}

const LOG_HZ: u32 = 20;
const LOG_PERIOD: Duration = Duration::from_micros(1_000_000 / LOG_HZ as u64);

fn logger_thread(state: Arc<State>) {
    let can_signals = configured_can_signals();
    let adc_channels = configured_adc_channels();

    // Empty so the schema is (re)written on the first tick and after every `next_log`.
    let mut current_name = String::new();

    let mut next_tick = std::time::Instant::now();

    loop {
        next_tick += LOG_PERIOD;

        if ACTIVE.load(Ordering::Relaxed) {
            let sources = snapshot(&can_signals, &adc_channels, &state);
            let mut sd = SD.lock();
            if sd.current_name() != current_name {
                if let Err(e) = write_schema(&mut *sd, &sources) {
                    log::error!("failed to write log schema: {e}");
                }
                current_name = sd.current_name();
            }
            if let Err(e) = write_row(&mut *sd, &sources) {
                log::warn!("failed to write log row: {e}");
            }
        }

        std::thread::sleep(next_tick.saturating_duration_since(std::time::Instant::now()));
    }
}
