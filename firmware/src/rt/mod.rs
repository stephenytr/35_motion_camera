//! RT plane (ARCHITECTURE §4): thin `#[ram]` ISR wrappers over the `logic` FSMs,
//! plus the single source of truth for "everything off".
//!
//! Constitution 3: ISRs are pure-and-tiny — no float, no allocation, no
//! logging, no blocking. All policy lives in `logic`; here we only apply
//! action records to hardware and pass events up via `try_send`.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::CriticalSectionMutex;

pub mod heartbeat;

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
pub fn arm_job(job: Job) {
    MAILBOX.lock(|m| *m.borrow_mut() = Some(job));
    heartbeat::kick();
}

/// Drive every actuator to its de-energized level. Called at boot, on panic, and
/// from the door / deadman ISRs. Everything here will be `#[ram]`.
pub fn safe_state() {
    // TODO: shutter LEDC duty 0, take-up LEDC duty 0, TMC5160 EN low,
    // heartbeat/deadman timers stopped.
}
