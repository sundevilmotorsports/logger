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
}

struct PrevSample {
    at_us: i64,
    idle_us: [u32; NUM_CORES],
}

#[derive(Default)]
pub struct ResourceMonitor {
    prev: Option<PrevSample>,
}

impl ResourceMonitor {
    /// CPU load is a delta since the last sample
    pub fn sample(&mut self) -> Resources {
        let now_us = unsafe { esp_timer_get_time() };
        let idle_us: [u32; NUM_CORES] =
            std::array::from_fn(|i| unsafe { ulTaskGetIdleRunTimeCounterForCore(i as i32) });

        let cpu_load: [f32; NUM_CORES] = match &self.prev {
            Some(prev) if now_us > prev.at_us => {
                let elapsed = (now_us - prev.at_us) as u32;
                std::array::from_fn(|i| {
                    let idle_delta = idle_us[i].wrapping_sub(prev.idle_us[i]);
                    let busy_pct = 100.0f32 - (idle_delta as f32 / elapsed as f32) * 100.0f32;
                    busy_pct.clamp(0.0, 100.0)
                })
            }
            _ => [0.0f32; NUM_CORES],
        };

        self.prev = Some(PrevSample { at_us: now_us, idle_us });

        Resources {
            heap_free: unsafe { esp_get_free_heap_size() },
            heap_min_free: unsafe { esp_get_minimum_free_heap_size() },
            cpu_load,
        }
    }
}
