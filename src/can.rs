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
    pub fn write_value(
        &self,
        raw: Option<&[u8]>,
        sink: &mut dyn std::io::Write,
    ) -> std::io::Result<()> {
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

    /// Physical value of `raw`, as logged; raw integer when `scale` is unset.
    pub fn value(&self, raw: &[u8]) -> f32 {
        let n = self.raw_int(raw) as f32;
        match self.scale {
            Some(scale) => scale * (n - self.offset),
            None => n,
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

impl Signals {
    /// Every signal, across all mux groups.
    pub fn iter(&self) -> impl Iterator<Item = &Signal> {
        let (fixed, groups): (&[Signal], &[SignalGroup]) = match self {
            Signals::Fixed(sigs) => (sigs, &[]),
            Signals::Muxed { groups, .. } => (&[], groups),
        };
        fixed
            .iter()
            .chain(groups.iter().flat_map(|g| g.signals.iter()))
    }
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

pub const BITRATE: u32 = 1_000_000;

fn frame_bits(extended: bool, data_len: usize) -> u32 {
    let overhead = if extended { 67 } else { 47 };
    overhead + 8 * data_len as u32
}

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
        match self.driver.receive(TickType::from(RECEIVE_TIMEOUT).into()) {
            Ok(frame) => {
                handle_frame(state, &frame);
                Ok(())
            }
            Err(e) if e.code() == ESP_ERR_TIMEOUT => Ok(()),
            Err(e) => Err(e),
        }
    }

    /// Streams `req.size` firmware bytes to `req.node` over CAN as the serial
    /// thread feeds them into `state.ota.staged`
    fn run_ota(&mut self, state: &State, req: OtaRequest) {
        *state.ota.progress.lock() = OtaProgress {
            active: true,
            sent: 0,
            total: req.size,
            result: None,
        };
        let finish = |result: u8| {
            let mut p = state.ota.progress.lock();
            p.active = false;
            p.result = Some(result);
        };

        // START (retry until ACKed)
        let mut start = [0u8; 8];
        start[0..4].copy_from_slice(&req.size.to_le_bytes());
        start[4..8].copy_from_slice(&req.crc.to_le_bytes());
        let start_id = sdm::can_id(sdm::Msg::OtaStart as u8, req.node);
        let mut started = false;
        for _ in 0..OTA_RETRIES {
            if self.send_ota_frame(state, start_id, &start).is_err() {
                continue;
            }
            match self.wait_ota_ack(state, req.node) {
                Some((_, 0)) => {
                    started = true;
                    break;
                }
                Some((_, status)) => return finish(status),
                None => continue,
            }
        }
        if !started {
            return finish(OTA_STATUS_TIMEOUT);
        }

        // DATA: ACK sliding window
        let data_id = sdm::can_id(sdm::Msg::OtaData as u8, req.node);
        let mut off: u32 = 0;
        let mut stalls = 0u32;
        while off < req.size {
            let window_end = (off + OTA_ACK_WINDOW * OTA_CHUNK as u32).min(req.size);

            // Wait for serial
            let mut starved_us = 0i64;
            while state.ota.staged.lock().end() < window_end {
                Ets.delay_us(OTA_FEED_POLL_US as u32);
                starved_us += OTA_FEED_POLL_US;
                if starved_us >= OTA_FEED_TIMEOUT_US {
                    return finish(OTA_STATUS_STARVED);
                }
            }

            let window_start = off;
            while off < window_end {
                let n = (window_end - off).min(OTA_CHUNK as u32) as usize;
                let mut frame = [0u8; 1 + OTA_CHUNK];
                frame[0] = (off / OTA_CHUNK as u32) as u8;
                {
                    let s = state.ota.staged.lock();
                    let start = (off - s.base) as usize;
                    for (i, slot) in frame[1..1 + n].iter_mut().enumerate() {
                        *slot = s.bytes[start + i];
                    }
                }
                if self
                    .send_ota_frame(state, data_id, &frame[..1 + n])
                    .is_err()
                {
                    break; // transient; the nodes next ACK rewinds
                }
                off += n as u32;
                Ets.delay_us(OTA_FRAME_GAP_US);
            }

            match self.wait_ota_ack(state, req.node) {
                Some((acked, 0)) => {
                    stalls = 0;
                    off = acked; // resume where the node actually is
                    {
                        let mut s = state.ota.staged.lock();
                        let advance = (acked.saturating_sub(s.base) as usize).min(s.bytes.len());
                        s.bytes.drain(..advance);
                        s.base += advance as u32;
                    }
                    state.ota.progress.lock().sent = acked;
                }
                Some((_, status)) => return finish(status),
                None => {
                    // No ACK
                    stalls += 1;
                    if stalls >= OTA_RETRIES {
                        return finish(OTA_STATUS_TIMEOUT);
                    }
                    off = window_start;
                }
            }
        }

        // END (retry until ACKed)
        let end_id = sdm::can_id(sdm::Msg::OtaEnd as u8, req.node);
        for _ in 0..OTA_RETRIES {
            if self.send_ota_frame(state, end_id, &[]).is_err() {
                continue;
            }
            if let Some((_, status)) = self.wait_ota_ack(state, req.node) {
                return finish(status);
            }
        }
        finish(OTA_STATUS_TIMEOUT);
    }

    fn send_ota_frame(&mut self, state: &State, id: u32, data: &[u8]) -> Result<(), EspError> {
        let frame = Frame::new(id, Flags::Extended.into(), data).unwrap();
        // Blocks while the TX queue is full; a timeout here means the bus is dead
        self.driver
            .transmit(&frame, TickType::from(OTA_TX_TIMEOUT).into())?;
        state
            .can_bits
            .fetch_add(frame_bits(true, data.len()), Ordering::Relaxed);
        Ok(())
    }

    /// Waits for an `OTA_ACK` from `node`. Other frames still land in `state`
    /// so logging keeps running through a transfer.
    fn wait_ota_ack(&mut self, state: &State, node: u8) -> Option<(u32, u8)> {
        let ack_id = sdm::can_id(sdm::Msg::OtaAck as u8, node);
        let deadline = unsafe { esp_timer_get_time() } + OTA_ACK_TIMEOUT_US;
        loop {
            let remaining_us = deadline - unsafe { esp_timer_get_time() };
            if remaining_us <= 0 {
                return None;
            }
            let timeout = Duration::from_micros(remaining_us as u64);
            match self.driver.receive(TickType::from(timeout).into()) {
                Ok(frame) => {
                    let d = frame.data();
                    if frame.is_extended() && frame.identifier() == ack_id && d.len() >= 5 {
                        state
                            .can_bits
                            .fetch_add(frame_bits(true, d.len()), Ordering::Relaxed);
                        return Some((u32::from_le_bytes([d[0], d[1], d[2], d[3]]), d[4]));
                    }
                    handle_frame(state, &frame);
                }
                Err(e) if e.code() == ESP_ERR_TIMEOUT => return None,
                Err(_) => return None,
            }
        }
    }

    pub fn spawn(self, state: Arc<State>) -> bool {
        std::thread::Builder::new()
            .stack_size(8192)
            .spawn(move || run(self, state))
            .inspect_err(|e| log::error!("can thread spawn failed: {e:?}"))
            .is_ok()
    }
}

/// Fold one received frame into `state`: heartbeats into `can_nodes`,
/// configured devices into `sensors.can`.
fn handle_frame(state: &State, frame: &Frame) {
    let raw_id = frame.identifier();
    let extended = frame.is_extended();
    let data = frame.data();
    state
        .can_bits
        .fetch_add(frame_bits(extended, data.len()), Ordering::Relaxed);

    if extended && sdm::can_id_msg(raw_id) == sdm::Msg::Heartbeat as u8 {
        let node = CanNode {
            device_type: data.first().copied().unwrap_or(0),
            last_seen_us: unsafe { esp_timer_get_time() },
        };
        state
            .sensors
            .can_nodes
            .lock()
            .insert(sdm::can_id_node(raw_id), node);
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
}

const OTA_CHUNK: usize = 7; // byte 0 is seq
const OTA_ACK_WINDOW: u32 = 16;
const OTA_FRAME_GAP_US: u32 = 500;
const OTA_ACK_TIMEOUT_US: i64 = 1_000_000;
const OTA_RETRIES: u32 = 5;
const OTA_TX_TIMEOUT: Duration = Duration::from_millis(10);
const OTA_FEED_POLL_US: i64 = 2_000;
const OTA_FEED_TIMEOUT_US: i64 = 10_000_000;
const OTA_STATUS_TIMEOUT: u8 = 0xFE;
const OTA_STATUS_STARVED: u8 = 0xFD;

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
            let ota_req = state.ota.request.lock().take();
            if let Some(req) = ota_req {
                log::info!("CAN OTA -> node 0x{:02X} ({} bytes)", req.node, req.size);
                can.run_ota(&state, req);
                continue;
            }
            can.poll_once(&state)?;
        }
    })
}
