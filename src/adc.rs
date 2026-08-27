use crate::configuration::CONFIGURATION;
use crate::state::State;
use esp_idf_svc::hal::spi::{SpiDeviceDriver, SpiDriver};
use esp_idf_svc::sys::EspError;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct AdcChannel {
    pub name: String,
    pub channel: u8,
    /// `Some` logs a scaled float; `None` logs the raw 12-bit count, matching a channel with no `processing` fn in the C++ logger.
    #[serde(default)]
    pub scale: Option<f32>,
    #[serde(default)]
    pub offset: f32,
}

pub enum AdcValue {
    Raw(u16),
    Float(f32),
}

impl AdcChannel {
    pub fn value(&self, raw: u16) -> AdcValue {
        match self.scale {
            Some(scale) => AdcValue::Float(scale * (raw as f32 - self.offset)),
            None => AdcValue::Raw(raw),
        }
    }
}

/// 0-2.5V or 0-5V unipolar input range, set once for the whole chip.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Range {
    R2_5V,
    R5V,
}

#[allow(dead_code)]
#[derive(Debug)]
pub enum AdcError {
    Spi(EspError),
    /// The address the chip echoed back didn't match the channel requested.
    ChannelMismatch {
        expected: u8,
        got: u8,
    },
}

/// Driver for the TI ADS7951 (12-bit, 8-channel, manual-mode SPI ADC).
/// Concrete over the board's one SPI device -- nothing else is plugged in here.
pub struct Adc {
    spi: SpiDeviceDriver<'static, Arc<SpiDriver<'static>>>,
    range: Range,
}

impl Adc {
    pub fn new(spi: SpiDeviceDriver<'static, Arc<SpiDriver<'static>>>, range: Range) -> Self {
        Self { spi, range }
    }

    /// Selects `channel` and returns its 12-bit raw conversion result
    pub fn read_channel(&mut self, channel: u8) -> Result<u16, AdcError> {
        let mut word = control_word(channel, self.range).to_be_bytes();
        self.spi
            .transfer_in_place(&mut word)
            .map_err(AdcError::Spi)?;
        self.spi
            .transfer_in_place(&mut word)
            .map_err(AdcError::Spi)?;

        let resp = u16::from_be_bytes(word);
        let got = ((resp >> 12) & 0x0F) as u8;
        if got != channel {
            return Err(AdcError::ChannelMismatch {
                expected: channel,
                got,
            });
        }
        Ok(resp & 0x0FFF)
    }

    pub fn spawn(self, state: Arc<State>) -> bool {
        std::thread::Builder::new()
            .stack_size(8192)
            .spawn(move || poll_loop(self, state))
            .inspect_err(|e| log::error!("adc thread spawn failed: {e:?}"))
            .is_ok()
    }
}

/// Manual-mode control word: DI15-12 = mode, DI11 = write-enable,
/// DI10-07 = channel address, DI06 = range, DI05 = powerdown (off),
/// DI04 = GPIO readback (off), DI03-00 = GPIO data (unused).
fn control_word(channel: u8, range: Range) -> u16 {
    const MANUAL_MODE: u16 = 0b0001 << 12;
    const WRITE_ENABLE: u16 = 1 << 11;

    let addr = (channel as u16 & 0x0F) << 7;
    let range_bit = match range {
        Range::R2_5V => 0,
        Range::R5V => 1 << 6,
    };

    MANUAL_MODE | WRITE_ENABLE | addr | range_bit
}

fn poll_loop(mut adc: Adc, state: Arc<State>) {
    log::info!("adc initialized");
    loop {
        let channels: Vec<u8> = CONFIGURATION
            .lock()
            .adc_channels
            .iter()
            .map(|c| c.channel)
            .collect();

        let mut latest = HashMap::with_capacity(channels.len());
        for ch in channels {
            match adc.read_channel(ch) {
                Ok(raw) => {
                    latest.insert(ch, raw);
                }
                Err(e) => log::warn!("ADC read error on channel {ch}: {e:?}"),
            }
        }

        state
            .status
            .adc
            .store(!latest.is_empty(), Ordering::Relaxed);
        *state.sensors.adc.lock() = latest;

        std::thread::sleep(Duration::from_millis(50));
    }
}
