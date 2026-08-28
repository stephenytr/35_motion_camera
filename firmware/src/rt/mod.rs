//! RT plane (ARCHITECTURE §4): thin `#[ram]` ISR wrappers over the `logic` FSMs,
//! plus the single source of truth for "everything off".
//!
//! TODO(ARCH §4.1): heartbeat ISR (timg0.0) applying `logic::frame_fsm::advance`.
//! TODO(ARCH §4.3): index GPIO ISR feeding `logic::index_watch::IndexWatch`.
//! TODO(ARCH §4.4): deadman ISR (timg1.1) — safe state + fault marker + WDT reboot.
//! TODO(ARCH §4.5): door interlock ISR (P3, in-ISR debounce re-check).
//! TODO(ARCH §3.2): non-blocking RMT transmit from ISR + status check.

/// Drive every actuator to its de-energized level. Called at boot, on panic, and
/// from the door / deadman ISRs. Everything here will be `#[ram]`.
pub fn safe_state() {
    // TODO: shutter LEDC duty 0, take-up LEDC duty 0, TMC5160 EN low,
    // heartbeat/deadman timers stopped.
}
