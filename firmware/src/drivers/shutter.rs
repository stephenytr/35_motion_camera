//! Shutter LEDC driver (ARCHITECTURE §3.3, §7): one 20 kHz channel, ISR-owned.
//!
//! Peak-and-hold waveform: 100% duty = pull-in, 25% = hold, 0% = release
//! (fails closed by spring at 0%/power loss).
//!
//! Configuration goes through the HAL once at boot (core 1 locals — the HAL
//! channel/timer wrappers are neither Send nor Sync, and drivers must be
//! constructed on the core that uses them). The duty transitions are raw
//! register writes mirroring the HAL's `set_duty` math (2^5 range, `<< 4`
//! field shift), so they are safe from the P2 ISRs that own the waveform.
//!
//! Bench: drives the user LED on GPIO 13 so the waveform is visible
//! (bright 4 ms pull, dim hold, off) without a scope.

use esp_hal::gpio::interconnect::PeripheralOutput;
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::channel::{self, ChannelIFace};
use esp_hal::ledc::timer::{self, TimerIFace};
use esp_hal::ledc::{HighSpeed, Ledc};
use esp_hal::time::Rate;

/// Duty resolution: 5-bit → 100% = 31 counts (ARCHITECTURE §3.3). The duty
/// register field is DUTY[24:4] integer + [3:0] fractional dither, so 5-bit
/// values sit at `<< 4`. 32 would exceed the 5-bit timer range, so the
/// maximum is 31.
const DUTY_MAX: u32 = 31;
const DUTY_SHIFT: u32 = 4; // integer part of the duty field starts at bit 4

/// Bench LED on GPIO 13 is active-low (lit when the pin is LOW), so the
/// output shows the complement of the true FET waveform. The driver's
/// `pull`/`hold`/`off` duty table stays true to ARCHITECTURE §3.3 (FET is
/// active-high); only the register write is inverted for the bench. Remove
/// this inversion when the real shutter FET lands.
const BENCH_INVERT: bool = true;

/// One-time setup on core 1: LEDC timer at 20 kHz (5-bit duty) + channel on
/// `pin`, starting released (0%). Wrappers are dropped; config is in hardware.
pub fn init(ledc: Ledc<'static>, pin: impl PeripheralOutput<'static>) {
    let mut t = ledc.timer::<HighSpeed>(timer::Number::Timer0);
    t.configure(timer::config::Config {
        duty: timer::config::Duty::Duty5Bit,
        clock_source: timer::HSClockSource::APBClk,
        frequency: Rate::from_khz(20),
    })
    .unwrap();

    // Channel config through the HAL; duty writes are raw (see below).
    let mut ch = ledc.channel::<HighSpeed>(channel::Number::Channel0, pin);
    ch.configure(channel::config::Config {
        timer: &t,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    })
    .unwrap();
}

/// Register write, ISR-safe: no locks, no HAL wrapper state.
///
/// The duty register is a *shadow*: hardware only transfers it into the live
/// compare when `CONF1.DUTY_START` is pulsed (auto-cleared). Without the
/// pulse, duty writes are silently ignored.
fn set_duty_raw(duty_value: u32) {
    let v = if BENCH_INVERT { DUTY_MAX - duty_value } else { duty_value };
    let ch = esp_hal::peripherals::LEDC::regs().hsch(0);
    ch.duty().write(|w| unsafe { w.duty().bits(v << DUTY_SHIFT) });
    ch.conf1().modify(|_, w| w.duty_start().set_bit());
}

/// Pull-in: 100% duty (heartbeat ISR at ExposeStart).
pub fn pull() {
    set_duty_raw(DUTY_MAX);
}

/// Hold: 25% duty (peak-hold ISR, 4 ms after pull-in).
pub fn hold() {
    set_duty_raw(DUTY_MAX * 25 / 100);
}

/// Release: 0% duty (exposure-end ISR).
pub fn off() {
    set_duty_raw(0);
}
