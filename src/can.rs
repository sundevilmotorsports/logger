use embedded_hal::{delay::DelayNs, spi::SpiDevice};
use mcp2518fd::memory::controller::fifo::PayloadSize;
use mcp2518fd::{
    id::{ExtendedId, Id},
    memory::controller::{configuration::OperationMode, fifo::FifoNumber, filter::FilterNumber},
    settings::{
        BitTimeConfiguration, DataBitTimeConfiguration, FifoConfiguration, FifoMode,
        FilterConfiguration, FilterMatchMode, IoConfiguration, NominalBitTimeConfiguration,
        OscillatorConfiguration, RxFifoConfiguration, Settings,
    },
    ConfigError, Error, MCP2518FD,
};
use std::collections::HashMap;

// --- Configuration types (define once at startup) ---

pub struct Signal {
    pub name: &'static str,
    pub start: usize,
    pub len: usize,
}

pub struct SignalGroup {
    pub type_val: u8,
    pub signals: &'static [Signal],
}

pub enum Signals {
    /// Same signals on every frame with this ID.
    Fixed(&'static [Signal]),
    /// Byte at `byte` is a type discriminator; selects which group to parse.
    Muxed {
        byte: usize,
        groups: &'static [SignalGroup],
    },
}

pub struct CanDevice {
    pub id: u32,
    pub extended: bool,
    pub fd: bool,
    pub signals: Signals,
}

// --- Output types ---

#[derive(Clone)]
pub struct ParsedSignal {
    pub name: &'static str,
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
                    .map(|g| g.signals)
                    .unwrap_or(&[])
            }
        };
        let signals = active_signals
            .iter()
            .filter(|s| s.start + s.len <= data.len())
            .map(|s| ParsedSignal {
                name: s.name,
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
}
