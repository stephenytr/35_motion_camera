//! Index watchdog (ARCHITECTURE §4.3) — step-loss detection wiring.
//!
//! The pure math lives in `logic::index_watch` (host-tested). Here it is
//! shadowed behind a critical-section mutex and driven two ways:
//!
//! - `frame_end_check()` — called by the heartbeat ISR at every counted
//!   *forward* frame: no index edge may be more than 200+slack full steps
//!   overdue.
//! - `on_edge()` — the real index-sensor GPIO ISR (P2) once the sensor is
//!   wired; until then `debug_edge()` / `debug_edge_at()` inject synthetic
//!   edges from task context for bench verification (same pattern as
//!   `rt::door::debug_force`).
//!
//! Any `Misaligned`/`MissedEdge` verdict latches the Jam path (SPECS §11:
//! stop cadence, shutter closed, driver off, `ERROR JAM`) — the same
//! recoverable immediate-safe-state sequence the door ISR uses, *not* the
//! deadman's reboot escalation.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::CriticalSectionMutex;
use logic::index_watch::{IndexVerdict, IndexWatch};

static WATCH: CriticalSectionMutex<RefCell<IndexWatch>> =
    CriticalSectionMutex::new(RefCell::new(IndexWatch::new()));

/// Watchdog arm. Disabled by default: the bench has no index sensor, and a
/// missing sensor is indistinguishable from 7 frames of real step loss
/// (200 steps ≈ 6.7 frames). The boot self-test enables it once the sensor
/// is wired (SPECS §11 "index present"); the bench hooks below enable it
/// explicitly for synthetic-edge validation.
static ENABLED: AtomicBool = AtomicBool::new(false);

/// Recover-job latch (SPECS §11 brownout): when set, the next accepted
/// index edge requests a park at the frame boundary — the recover creep
/// job runs with `frames: None`, so this is its stop condition.
static RECOVER_PENDING: AtomicBool = AtomicBool::new(false);

/// Arm/disarm the recover stop (director, at Recover job start).
pub fn arm_recover_stop(on: bool) {
    RECOVER_PENDING.store(on, Ordering::SeqCst);
}

/// Unused until the index-sensor boot self-test (SPECS §11) lands.
#[allow(dead_code)]
pub fn set_enabled(on: bool) {
    ENABLED.store(on, Ordering::Relaxed);
}

#[allow(dead_code)]
pub fn enabled() -> bool {
    ENABLED.load(Ordering::Relaxed)
}

/// Run a verdict through the fault path (ISR- and task-callable). No
/// logging here: the heartbeat calls this at P2 and §9 rule 2 forbids logs
/// in ISRs — the supervisor logs the received Fault event.
fn apply_verdict(v: IndexVerdict) {
    match v {
        IndexVerdict::Ok => {}
        IndexVerdict::Misaligned(_) | IndexVerdict::MissedEdge => {
            // A Recover job that faults before reaching a good edge must not
            // leave RECOVER_PENDING set — it would otherwise survive into
            // whatever unrelated job runs next and spuriously stop it at
            // that job's first accepted index edge (accept_edge() below is
            // the only other place this clears).
            RECOVER_PENDING.store(false, Ordering::SeqCst);
            crate::rt::safe_state();
            crate::rt::heartbeat::halt();
            crate::rt::shutter::disarm();
            // Same as the door path: stopping the heartbeat without
            // disarming the deadman would guarantee a watchdog reboot.
            crate::rt::deadman::disarm();
            crate::fault::raise(logic::interlock::ErrorCode::Jam);
        }
    }
}

/// Frame-end arrival check (heartbeat ISR, forward frames only). Reads the
/// RT position accumulator directly — no task involvement. Returns the
/// verdict WITHOUT applying it: the heartbeat ISR holds the heartbeat
/// timer's CS lock for its whole body, so `apply_verdict`'s `halt()` would
/// self-nest — the ISR handles a fault verdict inline instead (see
/// heartbeat.rs).
pub fn frame_end_check() -> IndexVerdict {
    if !ENABLED.load(Ordering::Relaxed) {
        return IndexVerdict::Ok;
    }
    if !crate::rt::position::direction().sign().is_positive() {
        return IndexVerdict::Ok; // rewind/creep-back moves the count backwards
    }
    let steps = crate::rt::position::usteps() / logic::consts::MICROSTEPS as i32;
    WATCH.lock(|w| w.borrow().check_frame_end(steps))
}

/// Shared acceptance path for a good edge (ISR- and task-callable).
fn accept_edge() {
    // No event: IndexTick was a no-op at the supervisor (status-only) and
    // the channel is per-core-CS — dropping the cross-core send removes
    // frame-rate-proportional traffic from the event channel entirely.
    if RECOVER_PENDING.swap(false, Ordering::SeqCst) {
        // Brownout recover: park at the frame boundary (SPECS §11). Safe
        // from a future P2 sensor ISR — request_stop only takes the cycle
        // CS mutex, and the heartbeat ISR is the same priority (no nest).
        crate::rt::heartbeat::request_stop();
    }
}

/// A real (or synthetic) index edge at the current commanded position.
/// The future index-sensor GPIO ISR (P2) calls this; until then the bench
/// hooks below exercise the identical path.
#[allow(dead_code)]
pub fn on_edge() {
    let steps = crate::rt::position::usteps() / logic::consts::MICROSTEPS as i32;
    let v = WATCH.lock(|w| w.borrow_mut().on_index_edge(steps));
    match v {
        IndexVerdict::Ok => accept_edge(),
        other => apply_verdict(other),
    }
}

/// TEMP bench hook: inject a perfectly-aligned synthetic index edge at the
/// current position (the real sensor's GPIO ISR replaces this).
#[allow(dead_code)]
pub fn debug_edge() {
    on_edge();
}

/// TEMP bench hook: inject an edge at an arbitrary commanded position to
/// exercise the misalignment path (e.g. `position_usteps() + 80`).
#[allow(dead_code)]
pub fn debug_edge_at(usteps: i32) {
    let steps = usteps / logic::consts::MICROSTEPS as i32;
    let v = WATCH.lock(|w| w.borrow_mut().on_index_edge(steps));
    match v {
        IndexVerdict::Ok => accept_edge(),
        other => apply_verdict(other),
    }
}
