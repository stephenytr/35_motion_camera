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
use esp_hal::peripherals::{GPIO12, GPIO21, GPIO23};

use logic::consts::TAKEUP_USTEPS_PER_FRAME;

/// LEDC APB clock (classic esp32).
const APB_HZ: u32 = 80_000_000;
/// 10-bit duty: 100% = 1023, 50% = 512.
const DUTY_BITS: u32 = 10;
const DUTY_HALF: u32 = 1 << (DUTY_BITS - 1);
/// Duty register field: integer part starts at bit 4 (fractional dither).
const DUTY_SHIFT: u32 = 4;

/// One-time setup of LEDC HS timer 1 + channel 1 and route the channel to
/// `step` via the GPIO matrix (called in the core-1 closure; config lives
/// in hardware — same pattern as drivers::shutter).
pub fn init_timer_and_channel(step: GPIO23<'static>) {
    step.connect_peripheral_to_output(esp_hal::gpio::OutputSignal::LEDC_HS_SIG1);
    let ledc = esp_hal::peripherals::LEDC::regs();
    // Timer 1: APB clock, 10-bit duty, paused with a placeholder divisor
    // until the first set_rate_fps.
    ledc.hstimer(1).conf().modify(|_, w| unsafe {
        w.tick_sel().bit(true);
        w.rst().clear_bit();
        w.pause().clear_bit();
        w.div_num().bits(78); // 80 MHz / (78 × 1024) ≈ 1 kHz placeholder
        w.duty_res().bits(DUTY_BITS as u8)
    });
    // Channel 1: bound to timer 1, output enabled, duty 0.
    ledc.hsch(1).hpoint().write(|w| unsafe { w.hpoint().bits(0) });
    ledc.hsch(1)
        .conf0()
        .modify(|_, w| unsafe { w.sig_out_en().set_bit().timer_sel().bits(1) });
    ledc.hsch(1).duty().write(|w| unsafe { w.duty().bits(0) });
    ledc.hsch(1)
        .conf1()
        .modify(|_, w| w.duty_start().set_bit());
}

pub struct Takeup {
    en: Output<'static>,
    dir: Output<'static>,
}

impl Takeup {
    pub fn new(en: GPIO12<'static>, dir: GPIO21<'static>) -> Self {
        Self {
            // ENN high = disabled at power-on.
            en: Output::new(en, Level::High, OutputConfig::default()),
            dir: Output::new(dir, Level::High, OutputConfig::default()),
        }
    }

    /// Set the takeup STEP rate for a film cadence (fps, f32 for the boost
    /// ramp's slewed cadence). Re-enables the driver — `safe_state()` pulls
    /// ENN high, so every job arm must bring it back.
    pub fn set_rate_fps(&mut self, fps: f32) {
        let hz = (fps.max(0.5) * TAKEUP_USTEPS_PER_FRAME as f32) as u32;
        let divisor = APB_HZ.div_ceil(hz * (1 << DUTY_BITS)).max(1);
        let ledc = esp_hal::peripherals::LEDC::regs();
        ledc.hstimer(1)
            .conf()
            .modify(|_, w| unsafe { w.div_num().bits(divisor) });
        // Duty register is absolute counts: re-latch 50% after the divisor
        // change (the old count would otherwise mean a different duty).
        ledc.hsch(1)
            .duty()
            .write(|w| unsafe { w.duty().bits(DUTY_HALF << DUTY_SHIFT) });
        ledc.hsch(1)
            .conf1()
            .modify(|_, w| w.duty_start().set_bit());
        self.en.set_low();
    }

    /// Stop the pulse train (park, stop, halt).
    pub fn off(&mut self) {
        let ledc = esp_hal::peripherals::LEDC::regs();
        ledc.hsch(1).duty().write(|w| unsafe { w.duty().bits(0) });
        ledc.hsch(1)
            .conf1()
            .modify(|_, w| w.duty_start().set_bit());
    }

    /// Film direction for the next/current job. `true` = forward.
    pub fn set_dir(&mut self, forward: bool) {
        self.dir.set_level(if forward { Level::High } else { Level::Low });
    }
}
