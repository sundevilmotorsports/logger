use crate::configuration::CONFIGURATION;
use embassy_time::Timer;
use embedded_hal::spi::SpiDevice;
use esp_idf_svc::hal::spi::{SpiDeviceDriver, SpiDriver};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::{Arc, LazyLock, Mutex};

pub type AdcBusType = AdcBus<SpiDeviceDriver<'static, Arc<SpiDriver<'static>>>>;

/// Latest raw 12-bit reading per ADC channel (0-7).
pub static LATEST_ADC: LazyLock<Mutex<HashMap<u8, u16>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));


#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct AdcChannel {
    pub name: String,
    pub channel: u8,
    #[serde(default = "default_scale")]
    pub scale: f32,
    #[serde(default)]
    pub offset: f32,
}

fn default_scale() -> f32 {
    1.0
}

impl AdcChannel {
    pub fn apply(&self, raw: u16) -> f32 {
        self.scale * (raw as f32 - self.offset)
    }
}

/// 0-2.5V or 0-5V unipolar input range, set once for the whole chip.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Range {
    R2_5V,
    R5V,
}

#[derive(Debug)]
pub enum AdcError<E> {
    Spi(E),
    /// The address the chip echoed back didn't match the channel requested.
    ChannelMismatch { expected: u8, got: u8 },
}

/// Driver for the TI ADS7951 (12-bit, 8-channel, manual-mode SPI ADC).
pub struct AdcBus<SPI> {
    spi: SPI,
    range: Range,
}

impl<SPI: SpiDevice> AdcBus<SPI> {
    pub fn new(spi: SPI, range: Range) -> Self {
        Self { spi, range }
    }

    /// Selects `channel` and returns its 12-bit raw conversion result.
    ///
    /// The ADS7951 pipelines by one SPI frame: the channel address written
    /// in a frame's SDI is sampled next frame and its result read back the
    /// frame after that. So selecting a channel and reading its result takes
    /// two 16-bit frames; we send the same request twice and take the
    /// second reply, which also lets us confirm the echoed channel address.
    pub fn read_channel(&mut self, channel: u8) -> Result<u16, AdcError<SPI::Error>> {
        let mut word = control_word(channel, self.range).to_be_bytes();
        self.spi.transfer_in_place(&mut word).map_err(AdcError::Spi)?;
        self.spi.transfer_in_place(&mut word).map_err(AdcError::Spi)?;

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

#[embassy_executor::task]
pub async fn adc_poll_task(bus: Arc<Mutex<AdcBusType>>) {
    loop {
        let channels: Vec<u8> = CONFIGURATION
            .lock()
            .unwrap()
            .adc_channels
            .iter()
            .map(|c| c.channel)
            .collect();

        let mut latest = HashMap::with_capacity(channels.len());
        {
            let mut guard = bus.lock().unwrap();
            for ch in channels {
                match guard.read_channel(ch) {
                    Ok(raw) => {
                        latest.insert(ch, raw);
                    }
                    Err(e) => log::warn!("ADC read error on channel {ch}: {e:?}"),
                }
            }
        }
        *LATEST_ADC.lock().unwrap() = latest;

        Timer::after_millis(50).await;
    }
}
