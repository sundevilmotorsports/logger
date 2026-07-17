use embassy_time::Timer;
use embedded_hal::{delay::DelayNs, spi::SpiDevice};
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
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::{Arc, LazyLock, Mutex};

pub type CanBusType = CanBus<SpiDeviceDriver<'static, SpiDriver<'static>>>;

pub static LATEST_FRAMES: LazyLock<Mutex<HashMap<u32, ParsedFrame>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

#[embassy_executor::task]
pub async fn can_poll_task(mut int_pin: PinDriver<'static, Input>, bus: Arc<Mutex<CanBusType>>) {
    loop {
        int_pin.wait_for_falling_edge().await.ok();

        let mut guard = bus.lock().unwrap();
        if let Err(e) = guard.poll_once() {
            log::warn!("CAN poll error: {:?}", e);
        }
        *LATEST_FRAMES.lock().unwrap() = guard.all_frames().clone();
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Signal {
    pub name: String,
    pub start: usize,
    pub len: usize,
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

#[derive(Clone)]
pub struct ParsedSignal {
    pub name: String,
    pub bytes: Vec<u8>,
}

#[derive(Clone)]
pub struct ParsedFrame {
    pub id: u32,
    pub extended: bool,
    pub fd: bool,
    pub signals: Vec<ParsedSignal>,
}

pub struct CanBus<SPI> {
    controller: MCP2518FD<SPI>,
    devices: Vec<CanDevice>,
    latest: HashMap<u32, ParsedFrame>,
}

impl<SPI: SpiDevice> CanBus<SPI> {
    /// Initializes the MCP2518FD in CAN FD mode (500 kbit/s nominal, 1 Mbit/s data).
    /// `delay` is only used here during chip mode transitions; not needed after init.
    pub fn new(spi: SPI, delay: &mut impl DelayNs) -> Result<Self, ConfigError> {
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
            latest: HashMap::new(),
        })
    }

    pub fn register_can_device(&mut self, device: CanDevice) {
        self.devices.push(device);
    }

    /// Latest received frame for every registered device, keyed by CAN ID.
    pub fn all_frames(&self) -> &HashMap<u32, ParsedFrame> {
        &self.latest
    }

    /// Drains all frames currently in the FIFO and updates `all_frames`.
    /// Unrecognized IDs are consumed and discarded so the FIFO never backs up.
    pub fn poll_once(&mut self) -> Result<(), Error> {
        loop {
            let Some(msg) = self.controller.rx_fifo_get_next(FifoNumber::Fifo1)? else {
                break;
            };
            let (raw_id, extended) = match msg.id() {
                Id::Standard(id) => (id.as_raw() as u32, false),
                Id::Extended(id) => (id.as_raw(), true),
            };
            if let Some(frame) = self.build_frame(raw_id, extended, msg.is_fd(), msg.data()) {
                self.latest.insert(raw_id, frame);
            }
        }
        Ok(())
    }

    /// Pops one frame and returns it, or None if the FIFO is empty or the ID is unregistered.
    pub fn receive(&mut self) -> Result<Option<ParsedFrame>, Error> {
        let Some(msg) = self.controller.rx_fifo_get_next(FifoNumber::Fifo1)? else {
            return Ok(None);
        };
        let (raw_id, extended) = match msg.id() {
            Id::Standard(id) => (id.as_raw() as u32, false),
            Id::Extended(id) => (id.as_raw(), true),
        };
        Ok(self.build_frame(raw_id, extended, msg.is_fd(), msg.data()))
    }

    fn build_frame(
        &self,
        raw_id: u32,
        extended: bool,
        fd: bool,
        data: &[u8],
    ) -> Option<ParsedFrame> {
        let device = self
            .devices
            .iter()
            .find(|d| d.id == raw_id && d.extended == extended)?;
        let active_signals: &[Signal] = match &device.signals {
            Signals::Fixed(sigs) => sigs,
            Signals::Muxed { byte, groups } => {
                let type_val = *data.get(*byte).unwrap_or(&0);
                groups
                    .iter()
                    .find(|g| g.type_val == type_val)
                    .map(|g| g.signals.as_slice())
                    .unwrap_or(&[])
            }
        };
        let signals = active_signals
            .iter()
            .filter(|s| s.start + s.len <= data.len())
            .map(|s| ParsedSignal {
                name: s.name.clone(),
                bytes: data[s.start..s.start + s.len].to_vec(),
            })
            .collect();
        Some(ParsedFrame {
            id: raw_id,
            extended,
            fd,
            signals,
        })
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
