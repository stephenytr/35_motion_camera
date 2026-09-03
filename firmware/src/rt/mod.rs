//! RT plane (ARCHITECTURE §4): thin `#[ram]` ISR wrappers over the `logic` FSMs,
//! plus the single source of truth for "everything off".
//!
//! Constitution 3: ISRs are pure-and-tiny — no float, no allocation, no
//! logging, no blocking. All policy lives in `logic`; here we only apply
//! action records to hardware and pass events up via `try_send`.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::CriticalSectionMutex;

pub mod deadman;
pub mod door;
pub mod heartbeat;
pub mod index;
pub mod position;
pub mod shutter;

/// One parameterized job (ARCHITECTURE §4.2), programmed by the director.
/// `frames: None` runs until stopped. `direction` selects motor polarity
/// and drives the RT position accumulator; the director programs the TMC
/// DIR pin to match at arm time.
pub struct Job {
    pub params: logic::frame_fsm::FrameParams,
    pub frames: Option<u32>,
    pub direction: logic::position::Direction,
}

/// Param mailbox (ARCHITECTURE §6): director writes a job here; the heartbeat
/// ISR applies it at the next frame start.
pub(crate) static MAILBOX: CriticalSectionMutex<RefCell<Option<Job>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// Program a job. Only valid between jobs (when the heartbeat is parked) —
/// the heartbeat applies the mailbox at the next FrameStart and then owns
/// the cadence. Kicking while a job runs would corrupt the phase.
///
/// The deadman timeout is programmed here per ARCHITECTURE §4.4: 2.5 × the
/// new frame period, floored at 100 ms.
pub fn arm_job(job: Job) {
    let timeout_us = (job.params.period_us.saturating_mul(5) / 2).max(100_000);
    deadman::set_timeout(timeout_us);
    // A full job replaces params — stale live updates (boost ramp) are void.
    heartbeat::clear_live_params();
    // A stale Recover-stop request from a previous, interrupted Recover
    // job (stopped/superseded before any index edge arrived) must not
    // leak into this new job and spuriously request a park at its first
    // accepted edge. Only `Command::Recover` re-arms this.
    index::arm_recover_stop(false);
    MAILBOX.lock(|m| *m.borrow_mut() = Some(job));
    heartbeat::kick();
}

/// Safety latch: once set (deadman / door ISR), the RT plane stops. The
/// heartbeat and shutter ISRs check it at entry; the RTC-watchdog feeder
/// stops feeding, so the chip reboots.
static SAFE: AtomicBool = AtomicBool::new(false);

/// Drive every actuator to its de-energized level (ARCHITECTURE §4.6). Called
/// at boot, on panic, and from the door / deadman ISRs. Raw register writes
/// only — no locks, no HAL state — so it works pre-init and at P3.
#[esp_hal::ram]
pub fn safe_state() {
    // Shutter + take-up LEDC channels to 0%, with the DUTY_START latch pulse.
    for ch in 0..2 {
        let ch = esp_hal::peripherals::LEDC::regs().hsch(ch);
        ch.duty().write(|w| unsafe { w.duty().bits(0) });
        ch.conf1().modify(|_, w| w.duty_start().set_bit());
    }
    // Lock-free latch: any already-fired shutter one-shot whose ISR is
    // still pending must not re-energize the driver after this (the stale
    // ISR window — rt::shutter module docs). The heartbeat's next arm
    // re-arms it.
    shutter::latch_disarmed();
    // Both TMC2209 ENN pins high = drivers disabled, motors freewheel
    // (SPECS §7.2). Raw GPIO poke: works pre-`esp_hal::init` and from P3
    // ISRs.
    let gpio = esp_hal::peripherals::GPIO::regs();
    for pin in [
        crate::consts::tmc2209_pins::TRANSPORT_EN,
        crate::consts::tmc2209_pins::TAKEUP_EN,
    ] {
        let mask = 1u32 << pin;
        unsafe {
            gpio.enable_w1ts().write(|w| w.bits(mask));
            gpio.out_w1ts().write(|w| w.bits(mask));
        }
    }
}

/// Latched safe state: actuators off *and* the RT plane / watchdog feeder are
/// told to stay down. Called by the deadman ISR (and later the door ISR).
#[esp_hal::ram]
pub fn latch_safe_state() {
    SAFE.store(true, Ordering::SeqCst);
    safe_state();
}

/// True once safe state is latched.
pub fn safe_active() -> bool {
    SAFE.load(Ordering::SeqCst)
}

/// Re-arm esp-hal's monotonic clock (the TIMG0 LACT counter).
///
/// esp-hal's `Instant` — which backs *every* esp-hal software deadline, e.g.
/// the OLED I2C transaction timeout — reads the LACT. esp-hal starts it in
/// `init()`, but every esp-hal `Timer::new` on TIMG0 (our heartbeat and
/// shutter one-shots) calls `PeripheralClockControl::reset(TIMG0)`, which
/// wipes the LACT config and freezes esp-hal time from that point on. Every
/// software timeout then silently never expires: a wedged I2C bus hangs the
/// blocking flush forever, starves the cooperative core-0 executor, and the
/// RTC watchdog reboots the chip.
///
/// Call this *after* the last TIMG0 timer setup. Mirrors esp-hal's
/// `time::implem::time_init`; APB is 80 MHz on the classic ESP32.
pub fn reinit_hal_clock() {
    const APB_HZ: u32 = 80_000_000;
    let tg0 = esp_hal::peripherals::TIMG0::regs();
    tg0.lactconfig().write(|w| unsafe { w.bits(0) });
    tg0.lactalarmhi().write(|w| unsafe { w.bits(u32::MAX) });
    tg0.lactalarmlo().write(|w| unsafe { w.bits(u32::MAX) });
    tg0.lactload().write(|w| unsafe { w.load().bits(1) });
    tg0.lactconfig().write(|w| {
        unsafe { w.divider().bits((APB_HZ / 16_000_000u32) as u16) };
        w.increase().bit(true);
        w.autoreload().bit(true);
        w.en().bit(true)
    });
}
