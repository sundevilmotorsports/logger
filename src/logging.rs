//! The logging thread. On a timer it snapshots [`State`], builds one fixed-width
//! row from every configured source (CAN signals, ADC channels, GNSS, power
//! monitor), and appends it to the current SD log file. Each file starts with a self-
//! describing header ([`sdm_utils::logfmt`]) so the desktop client can decode it
//! without the config.

use crate::adc::{AdcChannel, AdcValue};
use crate::can::{Signal, SignalValue, Signals};
use crate::configuration::CONFIGURATION;
use crate::gnss::Fix;
use crate::ina260::PowerReading;
use crate::sd::SdCard;
use crate::state::State;
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::sys::{esp_timer_get_time, EspError};
use sdm_utils::logfmt::{ColType, Schema};
use std::collections::HashMap;
use std::io::{self, Write};
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

fn col_type(scale: Option<f32>, raw_width: u8) -> ColType {
    if scale.is_some() {
        ColType::F32
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
    latest: &'a HashMap<String, Vec<u8>>,
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
    latest: &'a HashMap<u8, u16>,
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

struct PowerColumns(Option<PowerReading>);

impl LogSource for PowerColumns {
    fn schema(&self, push: &mut dyn FnMut(&str, ColType)) {
        push("amp_Batt", ColType::F32);
        push("v_batt", ColType::F32);
    }

    fn write_row(&self, sink: &mut dyn Write) -> io::Result<()> {
        let r = self.0.unwrap_or_default();
        sink.write_all(&(r.current_raw as f32 * 1.25).to_le_bytes())?;
        sink.write_all(&(r.voltage_raw as f32 * 1.25 / 1000.0).to_le_bytes())
    }
}

struct GpsColumns(Option<Fix>);

impl LogSource for GpsColumns {
    fn schema(&self, push: &mut dyn FnMut(&str, ColType)) {
        push("gps_Lat", ColType::F32);
        push("gps_Long", ColType::F32);
        push("alt", ColType::F32);
        push("sats", ColType::Raw(1));
        push("gps_fix", ColType::Raw(1));
        push("v_car_gps", ColType::F32);
        push("gps_heading", ColType::F32);
    }

    fn write_row(&self, sink: &mut dyn Write) -> io::Result<()> {
        let (lat, lon, alt_m, sats, quality, speed_kmh, heading_deg) = match &self.0 {
            Some(fix) => (
                fix.lat,
                fix.lon,
                fix.alt_m,
                fix.sats,
                fix.quality,
                fix.speed_kmh,
                fix.heading_deg,
            ),
            None => (0.0, 0.0, 0.0, 0, 0, 0.0, 0.0),
        };
        sink.write_all(&(lat as f32).to_le_bytes())?;
        sink.write_all(&(lon as f32).to_le_bytes())?;
        sink.write_all(&(alt_m as f32).to_le_bytes())?;
        sink.write_all(&[sats, quality])?;
        sink.write_all(&(speed_kmh as f32).to_le_bytes())?;
        sink.write_all(&(heading_deg as f32).to_le_bytes())
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

/// Per-tick scratch, kept across ticks so the steady state allocates nothing
#[derive(Default)]
struct Scratch {
    can: HashMap<String, Vec<u8>>,
    adc: HashMap<u8, u16>,
    row: Vec<u8>,
}

impl Scratch {
    /// Copy the latest sensor readings out from under their locks
    fn refresh(&mut self, state: &State) -> (Option<Fix>, Option<PowerReading>) {
        self.can.clone_from(&state.sensors.can.lock());
        self.adc.clone_from(&state.sensors.adc.lock());
        let gps = state.sensors.gps.lock().clone();
        let power = *state.sensors.power.lock();
        (gps, power)
    }
}

fn build_schema(sources: &[&dyn LogSource]) -> Schema {
    let mut schema = Schema::new();
    schema.push("timestamp", ColType::Raw(8));
    {
        let mut push = |name: &str, ty: ColType| schema.push(name, ty);
        for src in sources {
            src.schema(&mut push);
        }
    }
    schema
}

fn write_row(mut sink: impl Write, sources: &[&dyn LogSource]) -> io::Result<()> {
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

const LOG_HZ: u32 = 100;
const LOG_PERIOD: Duration = Duration::from_micros(1_000_000 / LOG_HZ as u64);

fn logger_thread(state: Arc<State>) -> ! {
    // Carried across supervisor restarts
    let mut prev_run_wrote = false;

    crate::supervisor::run(move || -> Result<(), EspError> {
        let mut can_signals = configured_can_signals();
        let mut adc_channels = configured_adc_channels();

        if std::mem::take(&mut prev_run_wrote) {
            SdCard::next_log().ok();
        }

        // The schema header goes at the top of every file: written on the first
        // active tick, and again after a `next_log` roll.
        let mut header_written = false;

        let mut scratch = Scratch::default();
        let mut next_tick = std::time::Instant::now();
        let mut last_sync = std::time::Instant::now();

        loop {
            next_tick += LOG_PERIOD;
            // A slow SD write can leave next_tick in the past
            let now = std::time::Instant::now();
            if now.saturating_duration_since(next_tick) > LOG_PERIOD {
                next_tick = now;
            }

            if state.logging.config_changed.swap(false, Ordering::Relaxed) {
                can_signals = configured_can_signals();
                adc_channels = configured_adc_channels();
                // Only roll if a file is actually underway
                if header_written {
                    SdCard::next_log().ok();
                    header_written = false;
                }
            }

            if state.logging.active.load(Ordering::Relaxed) {
                let (gps, power) = scratch.refresh(&state);
                let sources: [&dyn LogSource; 4] = [
                    &CanColumns {
                        signals: &can_signals,
                        latest: &scratch.can,
                    },
                    &AdcColumns {
                        channels: &adc_channels,
                        latest: &scratch.adc,
                    },
                    &GpsColumns(gps),
                    &PowerColumns(power),
                ];

                // A failed SD write means the card is gone or wedged
                let write = |data: &[u8]| -> Result<(), EspError> {
                    SdCard::write(data)
                        .inspect_err(|_| state.status.sd.store(false, Ordering::Relaxed))
                };

                if !header_written {
                    write(&build_schema(&sources).encode_header())?;
                    header_written = true;
                }

                scratch.row.clear();
                if let Err(e) = write_row(&mut scratch.row, &sources) {
                    log::warn!("failed to build log row: {e}");
                } else {
                    write(&scratch.row)?;
                    prev_run_wrote = true;
                    state.status.sd.store(true, Ordering::Relaxed);
                }

                // Flush the write buffer to storage about once a second so a
                // power loss costs at most that
                if last_sync.elapsed() >= Duration::from_secs(1) {
                    SdCard::sync()
                        .inspect_err(|_| state.status.sd.store(false, Ordering::Relaxed))?;
                    last_sync = std::time::Instant::now();
                }
            }

            std::thread::sleep(next_tick.saturating_duration_since(std::time::Instant::now()));
        }
    })
}
