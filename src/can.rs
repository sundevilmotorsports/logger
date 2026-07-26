use embedded_hal::delay::DelayNs;
use esp_idf_svc::hal::gpio::{Input, PinDriver};
use esp_idf_svc::hal::spi::{SpiDeviceDriver, SpiDriver};
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
use crate::state::SensorState;
use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex};

#[embassy_executor::task]
pub async fn can_poll_task(
    mut int_pin: PinDriver<'static, Input>,
    bus: Arc<Mutex<CanBus>>,
    state: Arc<SensorState>,
) {
    loop {
        int_pin.wait_for_falling_edge().await.ok();

        let updates = {
            let mut guard = bus.lock().unwrap();
            match guard.poll_once() {
                Ok(updates) => updates,
                Err(e) => {
                    log::warn!("CAN poll error: {:?}", e);
                    continue;
                }
            }
        };
        if !updates.is_empty() {
            let mut latest = state.can_signals.lock().unwrap();
            for (name, bytes) in updates {
                latest.insert(name, bytes);
            }
        }
    }
}

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
    /// `Some` logs `scale * (raw - offset)` as a float; `None` logs the raw
    /// bytes verbatim, same as a signal with no `processing` fn in the C++ logger.
    #[serde(default)]
    pub scale: Option<f32>,
    #[serde(default)]
    pub offset: f32,
}

pub enum SignalValue {
    Raw(Vec<u8>),
    Float(f32),
}

impl Signal {
    pub fn value(&self, raw: Option<&[u8]>) -> SignalValue {
        match self.scale {
            Some(scale) => {
                let n = raw.map(|r| self.raw_int(r)).unwrap_or(0);
                SignalValue::Float(scale * (n as f32 - self.offset))
            }
            None => {
                let mut buf = vec![0u8; self.len];
                if let Some(r) = raw {
                    let n = r.len().min(self.len);
                    buf[..n].copy_from_slice(&r[..n]);
                }
                SignalValue::Raw(buf)
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

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CanDevice {
    pub id: u32,
    pub extended: bool,
    pub fd: bool,
    pub signals: Signals,
}

/// Concrete over the board's one real SPI device rather than generic --
/// nothing else is ever plugged in here.
pub struct CanBus {
    controller: MCP2518FD<SpiDeviceDriver<'static, Arc<SpiDriver<'static>>>>,
    devices: Vec<CanDevice>,
}

impl CanBus {
    /// Initializes the MCP2518FD in CAN FD mode (500 kbit/s nominal, 1 Mbit/s data).
    /// `delay` is only used here during chip mode transitions; not needed after init.
    pub fn new(
        spi: SpiDeviceDriver<'static, Arc<SpiDriver<'static>>>,
        delay: &mut impl DelayNs,
    ) -> Result<Self, ConfigError> {
        let mut controller = MCP2518FD::new(spi);
        controller.reset().map_err(ConfigError::Other)?;
        controller.configure(
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

        controller
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

        controller
            .configure_fifo(
                FifoNumber::Fifo2,
                FifoConfiguration::new(
                    1,
                    PayloadSize::Bytes64,
                    FifoMode::Transmit(TxFifoConfiguration::new(0)),
                ),
            )
            .map_err(ConfigError::Other)?;

        controller
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
        controller.set_op_mode(OperationMode::NormalCanFD, delay)?;

        Ok(Self {
            controller,
            devices: Vec::new(),
        })
    }

    pub fn register_can_device(&mut self, device: CanDevice) {
        self.devices.push(device);
    }

    /// Drains the FIFO, returning every (signal name, raw bytes) update.
    /// Unrecognized IDs are discarded so the FIFO never backs up.
    pub fn poll_once(&mut self) -> Result<Vec<(String, Vec<u8>)>, Error> {
        let mut updates = Vec::new();
        loop {
            let Some(msg) = self.controller.rx_fifo_get_next(FifoNumber::Fifo1)? else {
                break;
            };
            let (raw_id, extended) = match msg.id() {
                Id::Standard(id) => (id.as_raw() as u32, false),
                Id::Extended(id) => (id.as_raw(), true),
            };
            self.collect_updates(raw_id, extended, msg.data(), &mut updates);
        }
        Ok(updates)
    }

    fn collect_updates(
        &self,
        raw_id: u32,
        extended: bool,
        data: &[u8],
        out: &mut Vec<(String, Vec<u8>)>,
    ) {
        let Some(device) = self
            .devices
            .iter()
            .find(|d| d.id == raw_id && d.extended == extended)
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
                out.push((sig.name.clone(), data[sig.start..sig.start + sig.len].to_vec()));
            }
        }
    }

    /// Verifies the TX/RX path using the controller's internal loopback mode:
    /// transmits a known frame and checks it comes back unchanged
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
}

#[derive(Debug)]
pub enum SelfTestError {
    Config(ConfigError),
    Comm(Error),
    /// No frame was looped back within the timeout.
    NoResponse,
    /// A frame was received but its ID or payload didn't match what was sent.
    Mismatch,
}
