//! Potentiometer inputs (bench): B10K pots as voltage dividers on ADC1,
//! polled at 100 ms. Each reading is quantized into whole-step buckets
//! (fps 3..=36, exposure 2..=1000 ms — the same full-step models as the
//! menu adjust) and a `UiEvent` is sent only when the bucket changes.
//! Anti-flicker is *hysteresis on the raw count* (a deadband around the
//! last locked position), not time-domain confirmation — a single sample
//! past the deadband responds immediately, and a pot resting at a bucket
//! boundary can't bounce between values.

use embassy_time::{Duration, Ticker};
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation, AdcPin};
use esp_hal::peripherals::{ADC1, GPIO34, GPIO35};
use log::info;

use super::{UiEvent, UI_EVENTS};
use logic::consts::{EXPOSURE_MAX_MS, EXPOSURE_MIN_MS, FPS_MAX, FPS_MIN};

const POLL_MS: u64 = 100;
/// 12-bit SAR; ~3.3 V span at 11 dB attenuation.
const ADC_MAX: u32 = 4095;
/// One bucket is ≥ ~124 raw counts (fps) / ≥ ~4 counts (exposure 2..1000);
/// a 20-count deadband kills wiper noise for fps, and a 1-count deadband
/// suffices for the fine-grained exposure pot (boundary flicker there is
/// within 1 ms, and the supervisor dedupes anyway).
const FPS_DEADBAND: u32 = 20;
const EXP_DEADBAND: u32 = 1;

struct Pot {
    last_raw: Option<u32>,
    last_value: Option<u32>,
    deadband: u32,
}

impl Pot {
    fn new(deadband: u32) -> Self {
        Self {
            last_raw: None,
            last_value: None,
            deadband,
        }
    }

    /// Returns `Some(value)` when the pot's whole-step bucket changed
    /// (first sample always wins: the pot's physical position is master).
    fn update(&mut self, raw: u32, value: u32) -> Option<u32> {
        let send = match (self.last_raw, self.last_value) {
            (None, _) => true,
            (Some(l_raw), Some(l_value)) => {
                raw.abs_diff(l_raw) > self.deadband && value != l_value
            }
            _ => unreachable!(),
        };
        if send {
            self.last_raw = Some(raw);
            self.last_value = Some(value);
            Some(value)
        } else {
            None
        }
    }
}

#[embassy_executor::task]
pub async fn pot_task(
    adc1: ADC1<'static>,
    fps_pin: GPIO34<'static>,
    exp_pin: GPIO35<'static>,
) {
    let mut cfg = AdcConfig::new();
    let mut fps_adc: AdcPin<GPIO34<'static>, ADC1> =
        cfg.enable_pin(fps_pin, Attenuation::_11dB);
    let mut exp_adc: AdcPin<GPIO35<'static>, ADC1> =
        cfg.enable_pin(exp_pin, Attenuation::_11dB);
    let mut adc = Adc::new(adc1, cfg);

    let mut fps = Pot::new(FPS_DEADBAND);
    let mut exp = Pot::new(EXP_DEADBAND);
    let mut ticker = Ticker::every(Duration::from_millis(POLL_MS));

    info!("ui: pots up — fps ADC1_CH6, exposure ADC1_CH7");
    loop {
        ticker.next().await;

        if let Ok(raw) = adc.read_oneshot(&mut fps_adc) {
            let v = (FPS_MIN as u32 + raw as u32 * (FPS_MAX as u32 - FPS_MIN as u32)
                / ADC_MAX) as u8;
            if let Some(v) = fps.update(raw as u32, v as u32) {
                let _ = UI_EVENTS.try_send(UiEvent::SetFps(v as u8));
            }
        }
        if let Ok(raw) = adc.read_oneshot(&mut exp_adc) {
            let v = EXPOSURE_MIN_MS
                + raw as u32 * (EXPOSURE_MAX_MS - EXPOSURE_MIN_MS) / ADC_MAX;
            if let Some(v) = exp.update(raw as u32, v) {
                let _ = UI_EVENTS.try_send(UiEvent::SetExposure(v));
            }
        }
    }
}
