//! MCP2518FD CAN FD controller over SPI. Interrupt-driven: waits for the INT
//! pin, drains the RX FIFO, and matches each frame against the configured
//! [`CanDevice`]s. Supports fixed-signal frames and muxed frames (a
//! discriminator byte selects a [`SignalGroup`]). Decoded signals land in
//! `State::sensors::can`.

use crate::configuration::CONFIGURATION;
use crate::state::{CanNode, OtaProgress, OtaRequest, State};
use embedded_hal::delay::DelayNs;
use esp_idf_svc::hal::can::{CanDriver, Flags, Frame};
use esp_idf_svc::hal::delay::{Ets, TickType};
use esp_idf_svc::sys::{esp_timer_get_time, EspError, ESP_ERR_TIMEOUT};
use sdm_utils as sdm;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Signal {
    pub name: String,
    pub start: usize,
    pub len: usize,
    /// Used only when `scale` is set, to decode the raw integer.
    #[serde(default)]
    pub signed: bool,
    #[serde(default)]
    pub big_endian: bool,
    /// `Some` logs a scaled float; `None` logs raw bytes, matching a signal with no `processing` fn in the C++ logger.
    #[serde(default)]
    pub scale: Option<f32>,
    #[serde(default)]
    pub offset: f32,
}

impl Signal {
    /// Write this signal's log column: a scaled `f32` if `scale` is set, else
    /// the raw bytes zero-padded/truncated to `len`. No allocation.
    pub fn write_value(&self, raw: Option<&[u8]>, sink: &mut dyn std::io::Write) -> std::io::Result<()> {
        match self.scale {
            Some(scale) => {
                let n = raw.map(|r| self.raw_int(r)).unwrap_or(0);
                sink.write_all(&(scale * (n as f32 - self.offset)).to_le_bytes())
            }
            None => {
                let raw = raw.unwrap_or(&[]);
                let n = raw.len().min(self.len);
                sink.write_all(&raw[..n])?;
                
                let mut pad = self.len - n;
                while pad > 0 {
                    let k = pad.min(8);
                    sink.write_all(&[0u8; 8][..k])?;
                    pad -= k;
                }
                Ok(())
            }
        }
    }

    fn raw_int(&self, raw: &[u8]) -> i64 {
        macro_rules! read {
            ($u:ty, $i:ty) => {{
                let mut buf = [0u8; std::mem::size_of::<$u>()];
                let n = raw.len().min(buf.len());
                buf[..n].copy_from_slice(&raw[..n]);
                let v = if self.big_endian {
                    <$u>::from_be_bytes(buf)
                } else {
                    <$u>::from_le_bytes(buf)
                };
                if self.signed {
                    v as $i as i64
                } else {
                    v as i64
                }
            }};
        }
        match self.len {
            1 => read!(u8, i8),
            2 => read!(u16, i16),
            4 => read!(u32, i32),
            _ => read!(u64, i64),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SignalGroup {
    pub type_val: u8,
    pub signals: Vec<Signal>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum Signals {
    /// Same signals on every frame with this ID.
    Fixed(Vec<Signal>),
    /// Byte at `byte` is a type discriminator; selects which group to parse.
    Muxed {
        byte: usize,
        groups: Vec<SignalGroup>,
    },
}

/// Which physical CAN network a device lives on.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Bus {
    #[default]
    Module,
    Engine,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CanDevice {
    pub id: u32,
    pub extended: bool,
    pub fd: bool,
    #[serde(default)]
    pub bus: Bus,
    pub signals: Signals,
}

const RECEIVE_TIMEOUT: Duration = Duration::from_millis(500);

pub struct Can {
    driver: CanDriver<'static>,
}

impl Can {
    pub fn new(driver: CanDriver<'static>) -> Self {
        Self { driver }
    }

    fn restart(&mut self) -> Result<(), EspError> {
        self.driver.stop().ok();
        self.driver.start()
    }

    /// Wait for one frame and fold it into `state`
    pub fn poll_once(&mut self, state: &State) -> Result<(), EspError> {
        let frame = match self.driver.receive(TickType::from(RECEIVE_TIMEOUT).into()) {
            Ok(frame) => frame,
            Err(e) if e.code() == ESP_ERR_TIMEOUT => return Ok(()),
            Err(e) => return Err(e),
        };

        let raw_id = frame.identifier();
        let extended = frame.is_extended();
        let data = frame.data();

        if extended && sdm::can_id_msg(raw_id) == sdm::Msg::Heartbeat as u8 {
            let node = CanNode {
                device_type: data.first().copied().unwrap_or(0),
                last_seen_us: unsafe { esp_timer_get_time() },
            };
            state.sensors.can_nodes.lock().insert(sdm::can_id_node(raw_id), node);
        } else {
            let config = CONFIGURATION.lock();
            collect_updates(
                &config.can_devices,
                raw_id,
                extended,
                data,
                &mut state.sensors.can.lock(),
            );
        }
        Ok(())
    }

    pub fn spawn(self, state: Arc<State>) -> bool {
        std::thread::Builder::new()
            .stack_size(8192)
            .spawn(move || run(self, state))
            .inspect_err(|e| log::error!("can thread spawn failed: {e:?}"))
            .is_ok()
    }
}

fn collect_updates(
    devices: &[CanDevice],
    raw_id: u32,
    extended: bool,
    data: &[u8],
    out: &mut HashMap<String, Vec<u8>>,
) {
    let Some(device) = devices
        .iter()
        .find(|d| d.id == raw_id && d.extended == extended)
    else {
        log::warn!("Device not found in config: {}", raw_id);
        return;
    };

    if device.id == 1000 {
        log::info!("Recv from ecu");
    }

    let active_signals: &[Signal] = match &device.signals {
        Signals::Fixed(sigs) => sigs,
        Signals::Muxed { byte, groups } => {
            let type_val = *data.get(*byte).unwrap_or(&0);
            match groups.iter().find(|g| g.type_val == type_val) {
                Some(g) => &g.signals,
                None => return,
            }
        }
    };
    for sig in active_signals {
        if sig.start + sig.len <= data.len() {
            let bytes = &data[sig.start..sig.start + sig.len];
            match out.get_mut(&sig.name) {
                Some(v) => {
                    v.clear();
                    v.extend_from_slice(bytes);
                }
                None => {
                    out.insert(sig.name.clone(), bytes.to_vec());
                }
            }
        }
    }
}

fn run(mut can: Can, state: Arc<State>) -> ! {
    crate::supervisor::run(move || -> Result<(), EspError> {
        state.status.can.store(false, Ordering::Relaxed);
        can.restart()?;
        state.status.can.store(true, Ordering::Relaxed);
        log::info!("can initialized");

        loop {
            can.poll_once(&state)?;
        }
    })
}
