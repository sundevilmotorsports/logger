//! MCP2518FD CAN FD controller over SPI. Interrupt-driven: waits for the INT
//! pin, drains the RX FIFO, and matches each frame against the configured
//! [`CanDevice`]s. Supports fixed-signal frames and muxed frames (a
//! discriminator byte selects a [`SignalGroup`]). Decoded signals land in
//! `State::sensors::can`.

use crate::configuration::CONFIGURATION;
use crate::state::{CanNode, OtaProgress, OtaRequest, State};
use embedded_hal::delay::DelayNs;
use esp_idf_svc::hal::delay::{Ets, FreeRtos};
use esp_idf_svc::hal::gpio::{Input, PinDriver};
use esp_idf_svc::hal::spi::{SpiDeviceDriver, SpiDriver};
use esp_idf_svc::sys::esp_timer_get_time;
use mcp2518fd::memory::chip::{DeviceIdRegister, OscillatorControlRegister};
use mcp2518fd::memory::controller::configuration::CanControlRegister;
use mcp2518fd::memory::controller::fifo::PayloadSize;
use mcp2518fd::{
    id::{ExtendedId, Id, StandardId},
    memory::controller::{configuration::OperationMode, fifo::FifoNumber, filter::FilterNumber},
    message::tx::TxMessage,
    settings::{
        BitTimeConfiguration, DataBitTimeConfiguration, FifoConfiguration, FifoMode,
        FilterConfiguration, FilterMatchMode, IoConfiguration, NominalBitTimeConfiguration,
        OscillatorConfiguration, RxFifoConfiguration, Settings, TxFifoConfiguration,
    },
    ConfigError, Error, MCP2518FD,
};
use sdm_utils as sdm;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::atomic::Ordering;
use std::sync::Arc;

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
    /// The sdm module network: heartbeats, node tracking, and OTA.
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

/// Nominal (arbitration-phase) bitrate of each bus.
pub const BITRATE: u32 = 500_000;
/// Number of buses feeding `State::can_bits` (Module + Engine).
pub const BUS_COUNT: u32 = 2;

/// Approximate on-wire bits for a frame, ignoring bit stuffing and the FD
/// data-phase speedup.
fn frame_bits(extended: bool, data_len: usize) -> u32 {
    let overhead = if extended { 67 } else { 47 };
    overhead + 8 * data_len as u32
}

/// Concrete over the board's one SPI device (nothing else is plugged in) and
/// owns `int_pin`, since the driver crate has no idea which GPIO it's wired to.
pub struct Can {
    controller: MCP2518FD<SpiDeviceDriver<'static, Arc<SpiDriver<'static>>>>,
    int_pin: PinDriver<'static, Input>,
    bus: Bus,
}

impl Can {
    pub fn new(
        spi: SpiDeviceDriver<'static, Arc<SpiDriver<'static>>>,
        int_pin: PinDriver<'static, Input>,
        bus: Bus,
    ) -> Self {
        Self {
            controller: MCP2518FD::new(spi),
            int_pin,
            bus,
        }
    }

    pub fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), ConfigError> {
        self.controller.reset().map_err(ConfigError::Other)?;
        // Give the oscillator time to start; SPI is clocked from SYSCLK
        delay.delay_ms(5);
        self.dump_spi_diagnostics();

        match self.controller.verify_spi_communications() {
            Ok(()) => log::info!("{:?} MCP2518FD SPI RAM echo OK", self.bus),
            Err(e) => log::error!("{:?} MCP2518FD SPI RAM echo failed: {e:?}", self.bus),
        }

        self.controller.configure(
            Settings {
                oscillator: OscillatorConfiguration::default(),
                io_configuration: IoConfiguration::new(),
                bit_time_configuration: BitTimeConfiguration::new(
                    NominalBitTimeConfiguration::RATE_500_KBIT,
                    DataBitTimeConfiguration::RATE_1_MBIT,
                ),
                tx_event_fifo: None,
                tx_queue: None,
                enable_time_based_counter: false,
                data_bits_to_match: None,
                enable_can_error_interrupts: false,
                enable_spi_error_interrupt: false,
                enable_ecc_error_interrupt: false,
            },
            delay,
        )?;

        self.controller
            .configure_fifo(
                FifoNumber::Fifo1,
                FifoConfiguration::new(
                    8,
                    PayloadSize::Bytes64,
                    FifoMode::Receive(
                        RxFifoConfiguration::new().with_fifo_not_empty_interrupt(true),
                    ),
                ),
            )
            .map_err(ConfigError::Other)?;

        self.controller
            .configure_fifo(
                FifoNumber::Fifo2,
                FifoConfiguration::new(
                    1,
                    PayloadSize::Bytes64,
                    FifoMode::Transmit(TxFifoConfiguration::new(0)),
                ),
            )
            .map_err(ConfigError::Other)?;

        self.controller
            .configure_filter(
                FilterNumber::Filter0,
                Some(FilterConfiguration {
                    buffer_pointer: FifoNumber::Fifo1,
                    mode: FilterMatchMode::Both,
                    filter_bits: Id::Extended(ExtendedId::ZERO),
                    mask_bits: Id::Extended(ExtendedId::ZERO),
                }),
            )
            .map_err(ConfigError::Other)?;
        self.controller
            .set_op_mode(OperationMode::NormalCanFD, delay)?;

        Ok(())
    }

    /// Logs raw register reads and one RAM echo so a dead SPI link can be told
    /// apart from a live chip. Expected after reset: DEVID 0x0000001X,
    /// OSC 0x00000460 (OSCRDY set), C1CON 0x04980760. All 0xFFFFFFFF means
    /// MISO is floating (chip unpowered, unclocked, or CS not reaching it);
    /// all 0x00000000 means MISO is held low.
    fn dump_spi_diagnostics(&mut self) {
        let bus = self.bus;
        let devid = self
            .controller
            .read_register::<DeviceIdRegister>()
            .map(u32::from);
        let osc = self
            .controller
            .read_register::<OscillatorControlRegister>()
            .map(u32::from);
        let c1con = self
            .controller
            .read_register::<CanControlRegister>()
            .map(u32::from);
        log::info!(
            "{bus:?} MCP2518FD raw: DEVID={:08X?} OSC={:08X?} C1CON={:08X?}",
            devid,
            osc,
            c1con
        );

        let pattern: u32 = 0xA5C3_5A3C;
        let mut read_back = [0u8; 4];
        let echo = self
            .controller
            .write_ram(0x400, &pattern.to_le_bytes())
            .and_then(|()| self.controller.read_ram(0x400, &mut read_back));
        log::info!(
            "{bus:?} MCP2518FD RAM echo: wrote {pattern:08X}, read {:08X} ({echo:?})",
            u32::from_le_bytes(read_back)
        );
    }

    /// Drains the FIFO and folds every frame into `state`; unrecognized IDs
    /// are discarded so it never backs up.
    pub fn poll_once(&mut self, state: &State) -> Result<(), Error> {
        while let Some(msg) = self.controller.rx_fifo_get_next(FifoNumber::Fifo1)? {
            let (raw_id, extended) = match msg.id() {
                Id::Standard(id) => (id.as_raw() as u32, false),
                Id::Extended(id) => (id.as_raw(), true),
            };
            handle_frame(state, self.bus, raw_id, extended, msg.data());
        }
        Ok(())
    }

    fn send_heartbeat(&mut self, state: &State) -> Result<(), Error> {
        let id = sdm::can_id(sdm::Msg::Heartbeat as u8, sdm::Node::Logger as u8);
        let payload = [sdm::DeviceType::Logger as u8];
        let msg = TxMessage::new_2_0(ExtendedId::new(id).unwrap(), &payload).unwrap();
        self.controller
            .tx_fifo_transmit_message(FifoNumber::Fifo2, &msg)?;
        state
            .can_bits
            .fetch_add(frame_bits(true, payload.len()), Ordering::Relaxed);
        Ok(())
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

    fn send_ota_frame(&mut self, state: &State, id: u32, data: &[u8]) -> Result<(), Error> {
        let msg = TxMessage::new_2_0(ExtendedId::new(id).unwrap(), data).unwrap();
        for _ in 0..OTA_TX_FIFO_RETRIES {
            match self
                .controller
                .tx_fifo_transmit_message(FifoNumber::Fifo2, &msg)
            {
                Err(Error::FifoFull) => Ets.delay_us(200),
                Ok(()) => {
                    state
                        .can_bits
                        .fetch_add(frame_bits(true, data.len()), Ordering::Relaxed);
                    return Ok(());
                }
                other => return other,
            }
        }
        // FIFO never drained
        Err(Error::FifoFull)
    }

    /// Waits for an `OTA_ACK` from `node`. Other frames still land in `state`
    /// so logging keeps running through a transfer.
    fn wait_ota_ack(&mut self, state: &State, node: u8) -> Option<(u32, u8)> {
        let ack_id = sdm::can_id(sdm::Msg::OtaAck as u8, node);
        let deadline = unsafe { esp_timer_get_time() } + OTA_ACK_TIMEOUT_US;
        while unsafe { esp_timer_get_time() } < deadline {
            match self.controller.rx_fifo_get_next(FifoNumber::Fifo1) {
                Ok(Some(msg)) => {
                    let (raw_id, extended) = match msg.id() {
                        Id::Standard(id) => (id.as_raw() as u32, false),
                        Id::Extended(id) => (id.as_raw(), true),
                    };
                    let d = msg.data();
                    if extended && raw_id == ack_id && d.len() >= 5 {
                        state
                            .can_bits
                            .fetch_add(frame_bits(true, d.len()), Ordering::Relaxed);
                        return Some((u32::from_le_bytes([d[0], d[1], d[2], d[3]]), d[4]));
                    }
                    handle_frame(state, self.bus, raw_id, extended, d);
                }
                Ok(None) => Ets.delay_us(200),
                Err(_) => return None,
            }
        }
        None
    }
}

/// Micros between the loggers own heartbeat broadcasts.
const HEARTBEAT_INTERVAL_US: i64 = 1_000_000;

const OTA_CHUNK: usize = 7; // byte 0 is seq
const OTA_ACK_WINDOW: u32 = 16;
const OTA_FRAME_GAP_US: u32 = 500;
const OTA_ACK_TIMEOUT_US: i64 = 1_000_000;
const OTA_RETRIES: u32 = 5;
/// ~10 ms (50 * 200 µs) waiting for a TX FIFO slot before declaring the bus dead
const OTA_TX_FIFO_RETRIES: u32 = 50;
const OTA_FEED_POLL_US: i64 = 2_000;
const OTA_FEED_TIMEOUT_US: i64 = 10_000_000; // serial stopped feeding bytes
const OTA_STATUS_TIMEOUT: u8 = 0xFE;
const OTA_STATUS_STARVED: u8 = 0xFD; // serial upload stalled

/// Fold one received frame into `state`: Module-bus heartbeats into
/// `can_nodes`, configured devices into `sensors.can`.
fn handle_frame(state: &State, bus: Bus, raw_id: u32, extended: bool, data: &[u8]) {
    state
        .can_bits
        .fetch_add(frame_bits(extended, data.len()), Ordering::Relaxed);

    if bus == Bus::Module && extended && sdm::can_id_msg(raw_id) == sdm::Msg::Heartbeat as u8 {
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
            bus,
            raw_id,
            extended,
            data,
            &mut state.sensors.can.lock(),
        );
    }
}

fn collect_updates(
    devices: &[CanDevice],
    bus: Bus,
    raw_id: u32,
    extended: bool,
    data: &[u8],
    out: &mut HashMap<String, Vec<u8>>,
) {
    let Some(device) = devices
        .iter()
        .find(|d| d.bus == bus && d.id == raw_id && d.extended == extended)
    else {
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

impl Can {
    /// Verifies TX/RX via internal loopback: sends a known frame, checks it comes back unchanged.
    pub fn self_test(&mut self, delay: &mut impl DelayNs) -> Result<(), SelfTestError> {
        const TEST_ID: u16 = 0x123;
        const TEST_DATA: [u8; 8] = [0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04];

        self.controller
            .set_op_mode(OperationMode::Configuration, delay)
            .map_err(SelfTestError::Config)?;
        self.controller
            .set_op_mode(OperationMode::InternalLoopback, delay)
            .map_err(SelfTestError::Config)?;

        let msg = TxMessage::new_2_0(StandardId::new(TEST_ID).unwrap(), &TEST_DATA).unwrap();
        let result = self
            .controller
            .tx_fifo_transmit_message(FifoNumber::Fifo2, &msg)
            .map_err(SelfTestError::Comm)
            .and_then(|()| {
                let mut outcome = Err(SelfTestError::NoResponse);
                for _ in 0..10 {
                    match self.controller.rx_fifo_get_next(FifoNumber::Fifo1) {
                        Ok(Some(rx)) => {
                            let id_matches =
                                matches!(rx.id(), Id::Standard(id) if id.as_raw() == TEST_ID);
                            outcome = if id_matches && rx.data() == TEST_DATA {
                                Ok(())
                            } else {
                                Err(SelfTestError::Mismatch)
                            };
                            break;
                        }
                        Ok(None) => delay.delay_ms(1),
                        Err(e) => {
                            outcome = Err(SelfTestError::Comm(e));
                            break;
                        }
                    }
                }
                outcome
            });

        self.controller
            .set_op_mode(OperationMode::Configuration, delay)
            .map_err(SelfTestError::Config)?;
        self.controller
            .set_op_mode(OperationMode::NormalCanFD, delay)
            .map_err(SelfTestError::Config)?;

        result
    }

    pub fn spawn(self, state: Arc<State>) -> bool {
        std::thread::Builder::new()
            .stack_size(8192)
            .spawn(move || run(self, state))
            .inspect_err(|e| log::error!("can thread spawn failed: {e:?}"))
            .is_ok()
    }
}

/// Init failure or a comm error partway through both land here, so either one
/// restarts the controller from scratch via `crate::supervisor::run`.
#[allow(dead_code)]
#[derive(Debug)]
enum RunError {
    Init(ConfigError),
    Pin(esp_idf_svc::sys::EspError),
    Comm(Error),
}

fn run(mut can: Can, state: Arc<State>) -> ! {
    let mut delay = FreeRtos;
    crate::supervisor::run(move || -> Result<(), RunError> {
        state.status.can.store(false, Ordering::Relaxed);
        can.init(&mut delay).map_err(RunError::Init)?;
        match can.self_test(&mut delay) {
            Ok(()) => log::info!("CAN self-test passed"),
            Err(e) => log::error!("CAN self-test failed: {:?}", e),
        }
        state.status.can.store(true, Ordering::Relaxed);
        log::info!("can initialized");

        let mut last_heartbeat_us = 0i64;
        loop {
            esp_idf_svc::hal::task::block_on(can.int_pin.wait_for_falling_edge())
                .map_err(RunError::Pin)?;

            can.poll_once(&state).map_err(RunError::Comm)?;
            let now = unsafe { esp_timer_get_time() };

            if can.bus == Bus::Module {
                let ota_req = state.ota.request.lock().take();
                if let Some(req) = ota_req {
                    log::info!("CAN OTA -> node 0x{:02X} ({} bytes)", req.node, req.size);
                    can.run_ota(&state, req);
                    last_heartbeat_us = unsafe { esp_timer_get_time() };
                    continue;
                }

                if now - last_heartbeat_us >= HEARTBEAT_INTERVAL_US {
                    last_heartbeat_us = now;
                    if let Err(e) = can.send_heartbeat(&state) {
                        log::warn!("heartbeat tx failed: {e:?}");
                    } else {
                        state.sensors.can_nodes.lock().insert(
                            sdm::Node::Logger as u8,
                            CanNode {
                                device_type: sdm::DeviceType::Logger as u8,
                                last_seen_us: now,
                            },
                        );
                    }
                }
            }
        }
    })
}

#[allow(dead_code)]
#[derive(Debug)]
pub enum SelfTestError {
    Config(ConfigError),
    Comm(Error),
    /// No frame was looped back within the timeout.
    NoResponse,
    /// A frame was received but its ID or payload didn't match what was sent.
    Mismatch,
}
