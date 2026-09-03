//! Takeup motor driver (ARCHITECTURE §3.2 note, SPECS §4.1): the takeup
//! spool runs as an open-loop follower of the film cadence — a STEP pulse
//! train whose frequency is the fps feedforward, with DIR set by the job
//! direction. LEDC high-speed timer 1 + channel 1 (the shutter owns timer 0
//! + channel 0), 10-bit duty at 50% = clean square wave.
//!
//! Raw register writes throughout (same pattern as the shutter driver): the
//! rate changes come from the director task and `safe_state()` must be able
//! to kill the channel from ISR context without locks.
//!
//! The rate is a ballpark by design: one frame of film is 7.125 mm and the
//! spool core is ~20 mm, so ~363 µsteps/frame (`TAKEUP_USTEPS_PER_FRAME`,
//! logic::consts). The mechanical takeup has a friction clutch for
//! compliance — the film tension loop is mechanical, the electronics only
//! need to stay roughly matched. `safe_state()` drives the channel duty to
//! zero and pulls ENN high, so a halted plane stops the takeup too; the
//! next `set_rate_fps` re-enables it.

use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::gpio::interconnect::PeripheralOutput;
use esp_hal::peripherals::{GPIO13, GPIO21, GPIO38};

use logic::consts::TAKEUP_USTEPS_PER_FRAME;

/// LEDC APB clock (classic esp32 and esp32-s3 both run APB at 80 MHz).
const APB_HZ: u32 = 80_000_000;
/// 10-bit duty: 100% = 1023, 50% = 512.
const DUTY_BITS: u32 = 10;
const DUTY_HALF: u32 = 1 << (DUTY_BITS - 1);
/// Duty register field: integer part starts at bit 4 (fractional dither).
const DUTY_SHIFT: u32 = 4;

/// One-time setup of LEDC timer 1 + channel 1 and route the channel to
/// `step` via the GPIO matrix (called in the core-1 closure; config lives
/// in hardware — same pattern as drivers::shutter).
///
/// S3 note (decision #33): the ESP32-S3's LEDC is the low-speed variant —
/// `esp-hal` calls it `LowSpeed`, and the register block uses `timer()` /
/// `ch()` accessors, `clk_div` instead of `div_num`, and requires a
/// `para_up` pulse to latch config changes. No `tick_sel` field: APB is
/// the only clock.
pub fn init_timer_and_channel(step: GPIO38<'static>) {
    step.connect_peripheral_to_output(esp_hal::gpio::OutputSignal::LEDC_LS_SIG1);
    let ledc = esp_hal::peripherals::LEDC::regs();
    // Timer 1: APB clock, 10-bit duty, paused with a placeholder divisor
    // until the first set_rate_fps.
    ledc.timer(1).conf().modify(|_, w| unsafe {
        w.rst().clear_bit();
        w.pause().clear_bit();
        // clk_div is Q8 fixed-point (esp-hal: `(src_freq << 8) / freq /
        // precision`); 20000 ≈ 1 kHz at 80 MHz / 1024 precision.
        w.clk_div().bits(20000);
        w.duty_res().bits(DUTY_BITS as u8);
        w.para_up().set_bit()
    });
    // Channel 1: bound to timer 1, output enabled, duty 0.
    ledc.ch(1).hpoint().write(|w| unsafe { w.hpoint().bits(0) });
    ledc.ch(1)
        .conf0()
        .modify(|_, w| unsafe {
            w.sig_out_en().set_bit();
            w.timer_sel().bits(1);
            w.para_up().set_bit()
        });
    ledc.ch(1).duty().write(|w| unsafe { w.duty().bits(0) });
    ledc.ch(1).conf1().modify(|_, w| w.duty_start().set_bit());
}

pub struct Takeup {
    en: Output<'static>,
    dir: Output<'static>,
}

impl Takeup {
    pub fn new(en: GPIO13<'static>, dir: GPIO21<'static>) -> Self {
        Self {
            // ENN high = disabled at power-on.
            en: Output::new(en, Level::High, OutputConfig::default()),
            dir: Output::new(dir, Level::High, OutputConfig::default()),
        }
    }

    /// Set the takeup STEP rate for a film cadence (fps, f32 for the boost
    /// ramp's slewed cadence). Re-enables the driver — `safe_state()` pulls
    /// ENN high, so every job arm must bring it back.
    ///
    /// `clk_div` is a Q8 fixed-point divisor (matches esp-hal's own
    /// `((src_freq as u64) << 8) / frequency / precision` — see
    /// `ledc/timer.rs::configure()` in esp-hal), not a plain integer. Omitting
    /// the `<< 8` makes the hardware run the timer 256x faster than intended
    /// — the coil sees a multi-MHz pulse train it can't follow (buzz/
    /// vibrate, no real rotation) instead of the few-kHz rate the cadence
    /// actually needs.
    pub fn set_rate_fps(&mut self, fps: f32) {
        let hz = (fps.max(0.5) * TAKEUP_USTEPS_PER_FRAME as f32) as u64;
        // div_num is Q8 fixed-point: 256 = divider 1.0 = the *slowest*
        // possible rate (~78 Hz at 10-bit duty), and the divisor shrinks as
        // the frequency rises (24 fps ≈ 8.7 kHz needs div ≈ 2-3). The old
        // clamp forced a 256 *minimum*, pinning the takeup at 78 Hz for
        // every cadence — the motor never visibly moved. Clamp low at 2
        // (div 1 would be 20 MHz of nonsense), high at the field max.
        let divisor = ((APB_HZ as u64) << 8)
            .div_ceil(hz * (1u64 << DUTY_BITS))
            .clamp(2, 0x3FFFF) as u32;
        #[cfg(feature = "debug-prints")]
        log::info!(
            "motor: takeup fps={fps} -> {hz} Hz, divisor={divisor} (LEDC LS timer1)",
        );
        let ledc = esp_hal::peripherals::LEDC::regs();
        ledc.timer(1).conf().modify(|_, w| unsafe {
            w.clk_div().bits(divisor);
            w.para_up().set_bit()
        });
        // Duty register is absolute counts: re-latch 50% after the divisor
        // change (the old count would otherwise mean a different duty).
        ledc.ch(1)
            .duty()
            .write(|w| unsafe { w.duty().bits(DUTY_HALF << DUTY_SHIFT) });
        ledc.ch(1)
            .conf1()
            .modify(|_, w| w.duty_start().set_bit());
        self.en.set_low();
        #[cfg(feature = "debug-prints")]
        log::info!("motor: takeup ENN LOW (enabled), STEP on GPIO38");
    }

    /// Stop the pulse train (park, stop, halt).
    pub fn off(&mut self) {
        let ledc = esp_hal::peripherals::LEDC::regs();
        ledc.ch(1).duty().write(|w| unsafe { w.duty().bits(0) });
        ledc.ch(1)
            .conf1()
            .modify(|_, w| w.duty_start().set_bit());
        #[cfg(feature = "debug-prints")]
        log::info!("motor: takeup off (duty 0)");
    }

    /// Film direction for the next/current job. `true` = forward.
    pub fn set_dir(&mut self, forward: bool) {
        self.dir.set_level(if forward { Level::High } else { Level::Low });
    }
}
