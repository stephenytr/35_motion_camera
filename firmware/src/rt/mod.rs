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
pub mod shutter;

/// One parameterized job (ARCHITECTURE §4.2), programmed by the director.
/// `frames: None` runs until stopped.
pub struct Job {
    pub params: logic::frame_fsm::FrameParams,
    pub frames: Option<u32>,
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
    // TMC2240/5160 ENN high = driver disabled, motor freewheels (SPECS §7.2).
    // Raw GPIO poke: works pre-`esp_hal::init` and from P3 ISRs.
    let gpio = esp_hal::peripherals::GPIO::regs();
    let mask = 1u32 << crate::consts::tmc_pins::EN;
    unsafe {
        gpio.enable_w1ts().write(|w| w.bits(mask));
        gpio.out_w1ts().write(|w| w.bits(mask));
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
