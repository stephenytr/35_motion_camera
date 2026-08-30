//! Heartbeat ISR (timg0.0, P2, core 1) — the phase authority (ARCHITECTURE
//! §3.1, §4.1). Fires twice per frame: FrameStart → ExposeStart.
//!
//! The ISR is a thin wrapper: it applies the pure `logic::frame_fsm::advance`
//! action record, re-arms itself, arms the shutter pair, and publishes events
//! with `try_send`. Nothing here blocks, logs, or uses floats.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::CriticalSectionMutex;
use esp_hal::interrupt::{InterruptHandler, Priority};
use esp_hal::time::Duration;
use esp_hal::timer::{timg::Timer, OneShotTimer};
use esp_hal::Blocking;
use logic::frame_fsm::{advance, FrameParams, Phase};

use super::MAILBOX;
use crate::drivers::shutter;
use crate::fault::{Event, EVENTS};

/// Runtime cycle state, owned exclusively by the heartbeat ISR.
/// One reader/writer, one core, one priority — the mutex exists to satisfy
/// Rust and to bound the cost, not to arbitrate contention.
pub struct Cycle {
    pub params: FrameParams,
    pub phase: Phase,
    pub remaining: Option<u32>,
    pub elapsed: u32,
}

static CYCLE: CriticalSectionMutex<RefCell<Cycle>> = CriticalSectionMutex::new(RefCell::new(Cycle {
    // Placeholder, replaced by the first arm_job before any firing.
    params: FrameParams {
        period_us: 41_666,
        pull_us: 22_916,
        settle_us: logic::consts::SETTLE_US,
        exp_us: 12_000,
        shutter_enabled: true,
    },
        phase: Phase::FrameStart,
        remaining: None,
        elapsed: 0,
    }));

static HEARTBEAT: CriticalSectionMutex<RefCell<Option<OneShotTimer<'static, Blocking>>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// Set by the heartbeat when a job parks (finite frames exhausted, Stop
/// request, or rewind-to-zero). The director polls-and-clears it to chain
/// script jobs (leader sequence, track-B setup) — §9 rule 5: only the ISR
/// writes, only the director clears.
static JOB_DONE: AtomicBool = AtomicBool::new(false);

/// Director-side poll: `true` once since the last call.
pub fn job_done_take() -> bool {
    JOB_DONE.swap(false, Ordering::SeqCst)
}

/// True between park and the next arm. Live param updates (boost ramp)
/// must no-op while parked: re-arming the deadman with no heartbeat to
/// feed it is a guaranteed watchdog reboot.
static PARKED: AtomicBool = AtomicBool::new(true);

pub fn is_parked() -> bool {
    PARKED.load(Ordering::SeqCst)
}

/// Params-only mailbox: `update_live_params` writes here so a boost ramp
/// can slew the timing of a *running* job without touching its frame
/// count, phase, or the deadman feed cadence. Applied by the ISR at
/// FrameStart, then a full job from MAILBOX (if any) wins over it.
static PARAMS_MAILBOX: CriticalSectionMutex<RefCell<Option<FrameParams>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// Bind the ISR on core 1. `timer` is constructed in `main` (core 0) and
/// moved here — handlers run on the core where they are *set up*, per
/// esp-hal, so this must be called inside the core-1 closure.
pub fn init(timer: Timer<'static>) {
    let mut hb = OneShotTimer::new(timer);
    hb.set_interrupt_handler(InterruptHandler::new(
        heartbeat_isr,
        Priority::Priority2, // ARCHITECTURE §2: frame timing at P2
    ));
    hb.listen();

    HEARTBEAT.lock(|slot| *slot.borrow_mut() = Some(hb));
}

/// Wake a parked heartbeat so a newly armed job starts immediately.
/// Called by `arm_job` only while parked (see `rt::arm_job` invariant).
pub fn kick() {
    PARKED.store(false, Ordering::SeqCst);
    HEARTBEAT.lock(|slot| {
        let mut slot = slot.borrow_mut();
        if let Some(hb) = slot.as_mut() {
            let _ = hb.schedule(Duration::from_micros(1));
        }
    });
}

/// Stop the heartbeat entirely. The deadman stays armed and will latch safe
/// state on expiry — used by the door ISR (M2d) and as the bench fault
/// injection. Idempotent. Marks the plane parked so live param updates
/// (boost ramp) cannot re-arm the deadman behind the halted heartbeat.
pub fn halt() {
    PARKED.store(true, Ordering::SeqCst);
    HEARTBEAT.lock(|slot| {
        let mut slot = slot.borrow_mut();
        if let Some(hb) = slot.as_mut() {
            hb.stop();
        }
    });
}

/// Request a clean park at the *next* frame boundary (Command::Stop,
/// ARCHITECTURE §6): forces `remaining = Some(0)`, which `advance()` turns
/// into a park action the next time it sees `Phase::FrameStart` — i.e. after
/// the in-flight frame's exposure finishes, never mid-pulldown. Safe to call
/// from the director task; the cycle state is behind the same
/// `CriticalSectionMutex` the ISR uses. A no-op if already parked.
pub fn request_stop() {
    CYCLE.lock(|cyc_cell| {
        cyc_cell.borrow_mut().remaining = Some(0);
    });
}

/// Live update of the running job's timing params without disturbing phase,
/// frame count, or the deadman feed cadence — used by the boost ramp
/// (Command::Boost, ARCHITECTURE §6) to slew fps smoothly. Unlike
/// `rt::arm_job`, this does **not** kick the heartbeat: it only overwrites
/// the mailbox, which the ISR already drains every `Phase::FrameStart`
/// during a running job, so the new params take effect at the next frame
/// boundary with no re-arm glitch.
/// Live update of the running job's timing params without disturbing phase,
/// frame count, or the deadman feed cadence — used by the boost ramp
/// (Command::Boost, ARCHITECTURE §6) to slew fps smoothly. No-op while
/// parked: without the heartbeat firing, re-arming the deadman here would
/// guarantee a watchdog reboot.
pub fn update_live_params(params: FrameParams) {
    if PARKED.load(Ordering::SeqCst) {
        return;
    }
    let timeout_us = (params.period_us.saturating_mul(5) / 2).max(100_000);
    super::deadman::set_timeout(timeout_us);
    PARAMS_MAILBOX.lock(|m| *m.borrow_mut() = Some(params));
}

/// Discard a pending live update (a newly armed job supersedes it).
pub fn clear_live_params() {
    PARAMS_MAILBOX.lock(|m| *m.borrow_mut() = None);
}

extern "C" fn heartbeat_isr() {
    HEARTBEAT.lock(|hb_cell| {
        let mut hb_slot = hb_cell.borrow_mut();
        let Some(hb) = hb_slot.as_mut() else {
            return;
        };
        hb.clear_interrupt();

        // Latched safe state: stop participating, no re-arm (ARCHITECTURE
        // §4.4). The deadman ISR already drove the actuators off.
        if super::safe_active() {
            return;
        }

        CYCLE.lock(|cyc_cell| {
            let mut cyc = cyc_cell.borrow_mut();

            // Apply pending timing updates at frame start, then a full
            // job (which replaces params, remaining, and direction — a new
            // arm wins over any stale live update).
            if cyc.phase == Phase::FrameStart {
                if let Some(p) = PARAMS_MAILBOX.lock(|m| m.borrow_mut().take()) {
                    cyc.params = p;
                }
                if let Some(job) = MAILBOX.lock(|m| m.borrow_mut().take()) {
                    cyc.params = job.params;
                    cyc.remaining = job.frames;
                    cyc.elapsed = 0;
                    super::position::set_direction(job.direction);
                }
            }

            let params = cyc.params; // Copy: split borrows cannot span a struct
            let actions = advance(cyc.phase, &params, &mut cyc.remaining);
            cyc.phase = match cyc.phase {
                Phase::FrameStart => Phase::ExposeStart,
                Phase::ExposeStart => Phase::FrameStart,
            };

            // Apply the action record — register writes only, no waits.
            if actions.rmt_kick {
                // Non-blocking STEP transmit for this frame's pulldown
                // (ARCHITECTURE §3.2). Busy → Fault::RmtBusy, cadence continues.
                crate::drivers::rmt_step::kick();
            }
            if actions.shutter_pull {
                shutter::pull();
            }
            if let Some(us) = actions.arm_hold_us {
                super::shutter::arm_hold(us);
            }
            if let Some(us) = actions.arm_exposure_us {
                super::shutter::arm_exposure(us);
            }

            if actions.frame_counted {
                cyc.elapsed += 1;
                // Film position accumulator (ARCHITECTURE §4.2): one frame
                // of film per counted frame, direction-aware, clamped at
                // the datum. Feeds rewind-to-zero and the index watchdog.
                super::position::advance_frame(super::position::direction());
                if !matches!(
                    super::index::frame_end_check(),
                    logic::index_watch::IndexVerdict::Ok
                ) {
                    // Step-loss JAM (ARCHITECTURE §4.3): the same immediate
                    // actions as the door path, done inline — `halt()`
                    // would self-nest inside this ISR's own timer lock.
                    super::safe_state();
                    super::shutter::disarm();
                    super::deadman::disarm();
                    PARKED.store(true, Ordering::SeqCst);
                    let _ = EVENTS.try_send(Event::Fault(logic::interlock::ErrorCode::Jam));
                    return;
                }
                let _ = EVENTS.try_send(Event::FrameDone(cyc.elapsed));
            }

            if actions.park {
                shutter::off();
                super::deadman::disarm();
                // Park completes the cycle: the next arm kicks into a
                // clean FrameStart (a leftover ExposeStart here would
                // count a phantom frame on the next job's first kick).
                cyc.phase = Phase::FrameStart;
                PARKED.store(true, Ordering::SeqCst);
                JOB_DONE.store(true, Ordering::SeqCst);
                if super::position::at_datum()
                    && super::position::direction() == logic::position::Direction::Reverse
                {
                    // Rewind-to-zero completed: film is back at the datum.
                    let _ = EVENTS.try_send(Event::CounterZero);
                }
                let _ = EVENTS.try_send(Event::JobComplete);
            } else if let Some(us) = actions.arm_heartbeat_us {
                // Feed the deadman before the next cadence step (ARCHITECTURE
                // §4.4): the watchdog stays covered a full timeout past any
                // firing while a job is active.
                super::deadman::feed();
                let _ = hb.schedule(Duration::from_micros(us as u64));
            }
        });
    });
}
