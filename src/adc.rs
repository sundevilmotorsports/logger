//! TI ADC128S102 SPI ADC: an 8-channel, single-ended, pipelined-conversion
//! chip (no manual channel-select/echo like the P4 board's ADS7951 had). Each
//! SPI word both selects the *next* channel to convert and returns the result
//! of the *previous* one, so all 8 channels are read as one sequential sweep
//! every cycle, matching `read_all_channels()` in the C firmware's `adc.c`.
//! Raw 16-bit results are stored unscaled, same as the C firmware.

use crate::state::State;
use esp_idf_svc::hal::spi::{SpiDeviceDriver, SpiDriver};
use esp_idf_svc::sys::EspError;
use std::collections::HashMap;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

type AdcSpi = SpiDeviceDriver<'static, SpiDriver<'static>>;

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub struct AdcChannel {
    pub name: String,
    pub channel: u8,
    /// `Some` logs a scaled float; `None` logs the raw 16-bit count, matching a channel with no `processing` fn in the C++ logger.
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

#[allow(dead_code)]
#[derive(Debug)]
pub enum AdcError {
    Spi(EspError),
}

/// Driver for the TI ADC128S102 (8-channel, pipelined SPI ADC).
/// Concrete over the board's one SPI device -- nothing else is plugged in here.
pub struct Adc {
    spi: AdcSpi,
}

impl Adc {
    pub fn new(spi: AdcSpi) -> Self {
        Self { spi }
    }

    /// One full sweep of all 8 channels. Because the chip pipelines by one
    /// conversion, this issues 9 transfers: select ch0, then for ch=1..=7
    /// select ch while reading back ch-1's result, then a final transfer
    /// (select ch0 again) to read back ch7's result.
    fn read_all_channels(&mut self) -> Result<[u16; 8], AdcError> {
        let mut results = [0u16; 8];

        let mut word = [0u8, 0u8];
        self.spi
            .transfer_in_place(&mut word)
            .map_err(AdcError::Spi)?;

        for ch in 1..=7u8 {
            word = [ch << 3, 0];
            self.spi
                .transfer_in_place(&mut word)
                .map_err(AdcError::Spi)?;
            results[(ch - 1) as usize] = u16::from_be_bytes(word);
        }

        word = [0, 0];
        self.spi
            .transfer_in_place(&mut word)
            .map_err(AdcError::Spi)?;
        results[7] = u16::from_be_bytes(word);

        Ok(results)
    }

    pub fn spawn(self, state: Arc<State>) -> bool {
        std::thread::Builder::new()
            .stack_size(8192)
            .spawn(move || poll_loop(self, state))
            .inspect_err(|e| log::error!("adc thread spawn failed: {e:?}"))
            .is_ok()
    }
}

const ADC_HZ: u32 = 500;
const ADC_PERIOD: Duration = Duration::from_micros(1_000_000 / ADC_HZ as u64);

fn poll_loop(mut adc: Adc, state: Arc<State>) -> ! {
    crate::supervisor::run(move || -> Result<(), AdcError> {
        state.status.adc.store(false, Ordering::Relaxed);
        log::info!("adc initialized");
        loop {
            let channels = adc.read_all_channels()?;

            let latest: HashMap<u8, u16> = channels
                .into_iter()
                .enumerate()
                .map(|(ch, v)| (ch as u8, v))
                .collect();

            state.status.adc.store(true, Ordering::Relaxed);
            *state.sensors.adc.lock() = latest;

            std::thread::sleep(ADC_PERIOD);
        }
    })
}
