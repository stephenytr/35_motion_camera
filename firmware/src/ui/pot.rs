//! FPS potentiometer input (bench): B10K pot as a voltage divider on an
//! ADC1 pin, polled at 100 ms. The raw reading is quantized into whole-fps
//! buckets (3..=36, the same full-step model as the menu adjust) and a
//! `UiEvent::SetFps` is sent only when the bucket changes. Anti-flicker is
//! *hysteresis on the raw count* (a deadband around the last locked
//! position), not time-domain confirmation — a single sample past the
//! deadband responds immediately, and a pot resting at a bucket boundary
//! can't bounce between values.

use embassy_time::{Duration, Ticker};
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::peripherals::{ADC1, GPIO34};
use log::info;

use super::{UiEvent, UI_EVENTS};
use logic::consts::{FPS_MAX, FPS_MIN};

const POLL_MS: u64 = 100;
/// 12-bit SAR; ~3.3 V span at 11 dB attenuation.
const ADC_MAX: u32 = 4095;
/// One fps bucket ≈ 124 raw counts. A 20-count deadband around the last
/// locked position kills wiper noise at a boundary without costing
/// response time (the send fires on the first sample that escapes it).
const DEADBAND: u32 = 20;

#[embassy_executor::task]
pub async fn pot_task(adc1: ADC1<'static>, pin: GPIO34<'static>) {
    let mut cfg = AdcConfig::new();
    let mut adc_pin = cfg.enable_pin(pin, Attenuation::_11dB);
    let mut adc = Adc::new(adc1, cfg);

    // (raw, fps) at the last accepted change. The deadband window only
    // slides on a real bucket change, so the hysteresis zone stays pinned
    // to the bucket boundary (not chasing the raw while you turn slowly).
    let mut lock: Option<(u32, u8)> = None;
    let mut ticker = Ticker::every(Duration::from_millis(POLL_MS));

    info!("ui: fps pot up on ADC1 (3–36 fps)");
    loop {
        ticker.next().await;

        let raw = match adc.read_oneshot(&mut adc_pin) {
            Ok(v) => v as u32,
            Err(_) => continue,
        };
        // Raw → 3..=36 fps, whole steps.
        let fps =
            (FPS_MIN as u32 + raw * (FPS_MAX as u32 - FPS_MIN as u32) / ADC_MAX) as u8;

        let send = match lock {
            None => true, // first sample: the pot's physical position wins
            Some((l_raw, l_fps)) => raw.abs_diff(l_raw) > DEADBAND && fps != l_fps,
        };
        if send {
            lock = Some((raw, fps));
            let _ = UI_EVENTS.try_send(UiEvent::SetFps(fps));
        }
    }
}
