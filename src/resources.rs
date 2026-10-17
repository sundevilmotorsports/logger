//! Free-heap, per-core CPU-load and CAN bus-load sampling, reported to the desktop client for
//! health monitoring. [`ResourceMonitor::sample`] diffs the FreeRTOS idle
//! counters (and the CAN bit counter) against the previous call, so load is
//! "since you last asked".

use crate::can;
use esp_idf_svc::hal::cpu::CORES;
use esp_idf_svc::sys::{
    esp_get_free_heap_size, esp_get_minimum_free_heap_size, esp_timer_get_time,
    ulTaskGetIdleRunTimeCounterForCore,
};
use serde::Serialize;

const NUM_CORES: usize = CORES as usize;

#[derive(Serialize)]
pub struct Resources {
    pub heap_free: u32,
    pub heap_min_free: u32,
    /// Percent busy (0-100) per core, since the previous sample.
    pub cpu_load: [f32; NUM_CORES],
    pub can_load: f32,
}

struct PrevSample {
    at_us: i64,
    idle_us: [u32; NUM_CORES],
    can_bits: u32,
}

#[derive(Default)]
pub struct ResourceMonitor {
    prev: Option<PrevSample>,
}

impl ResourceMonitor {
    /// CPU and CAN load are deltas since the last sample
    pub fn sample(&mut self, can_bits: u32) -> Resources {
        let now_us = unsafe { esp_timer_get_time() };
        let idle_us: [u32; NUM_CORES] =
            std::array::from_fn(|i| unsafe { ulTaskGetIdleRunTimeCounterForCore(i as i32) });

        let (cpu_load, can_load) = match &self.prev {
            Some(prev) if now_us > prev.at_us => {
                let elapsed = (now_us - prev.at_us) as u32;
                let cpu_load: [f32; NUM_CORES] = std::array::from_fn(|i| {
                    let idle_delta = idle_us[i].wrapping_sub(prev.idle_us[i]);
                    let busy_pct = 100.0f32 - (idle_delta as f32 / elapsed as f32) * 100.0f32;
                    busy_pct.clamp(0.0, 100.0)
                });

                let bits_delta = can_bits.wrapping_sub(prev.can_bits);
                let bus_bits = elapsed as f32 * can::BITRATE as f32 / 1_000_000.0f32;
                let can_load = (bits_delta as f32 / bus_bits * 100.0f32).clamp(0.0, 100.0);

                (cpu_load, can_load)
            }
            _ => ([0.0f32; NUM_CORES], 0.0f32),
        };

        self.prev = Some(PrevSample {
            at_us: now_us,
            idle_us,
            can_bits,
        });

        Resources {
            heap_free: unsafe { esp_get_free_heap_size() },
            heap_min_free: unsafe { esp_get_minimum_free_heap_size() },
            cpu_load,
            can_load,
        }
    }
}
