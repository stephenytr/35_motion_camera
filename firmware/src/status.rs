//! Field-atomic shared status (ARCHITECTURE §5.2): single writer per field,
//! lock-free reads for the UI.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

/// Top-level states (SPECS §8.4 / ARCHITECTURE §8).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum State {
    Idle = 0,
    Thread = 1,
    Run = 2,
    Single = 3,
    Inch = 4,
    Rewind = 5,
    Script = 6,
    Door = 7,
    Error = 8,
}

impl State {
    pub const fn from_u32(v: u32) -> Option<Self> {
        match v {
            0 => Some(State::Idle),
            1 => Some(State::Thread),
            2 => Some(State::Run),
            3 => Some(State::Single),
            4 => Some(State::Inch),
            5 => Some(State::Rewind),
            6 => Some(State::Script),
            7 => Some(State::Door),
            8 => Some(State::Error),
            _ => None,
        }
    }
}

pub struct Status {
    pub frames_exposed: AtomicU32,
    /// Cumulative film frames exposed (ISR-incremented, supervisor-polled).
    /// Distinct from `frames_exposed` (job-local elapsed, UI display).
    pub exposed_count: AtomicU32,
    /// The supervisor's adjusted exposure counter (rewind-decremented,
    /// film-end-checked) — published here for the UI display.
    pub counter_exposed: AtomicU32,
    pub state: AtomicU32,
    pub door_open: AtomicBool,
    /// Boost active (supervisor-owned; UI displays it).
    pub boost: AtomicBool,
    pub vbat_mv: AtomicU32,
    /// Low-battery warn latch (SPECS §11: warn ≤ 19.8 V, no stop). Distinct
    /// from `fault` — `CriticalBattery` (≤ 18.3 V) is the one that stops.
    pub batt_warn: AtomicBool,
    /// 0 = none, else ErrorCode discriminant + payload (ARCHITECTURE §5.3).
    pub fault: AtomicU32,
    /// RMT STEP diagnostics (ISR/director-written on core 1, polled on core
    /// 0): transfer kicks, kicks that found the slot still busy (RmtBusy
    /// raised), table rebuilds that found a transfer genuinely in flight
    /// (the "ramp tick deferred" case), and S3 mid-transfer threshold
    /// refills serviced by the P2 ISR.
    pub rmt_kicks: AtomicU32,
    pub rmt_busy: AtomicU32,
    pub rmt_poll_false: AtomicU32,
    pub rmt_refills: AtomicU32,
}

impl Status {
    pub const fn new() -> Self {
        Self {
            frames_exposed: AtomicU32::new(0),
            exposed_count: AtomicU32::new(0),
            counter_exposed: AtomicU32::new(0),
            state: AtomicU32::new(State::Idle as u32),
            door_open: AtomicBool::new(false),
            boost: AtomicBool::new(false),
            vbat_mv: AtomicU32::new(0),
            batt_warn: AtomicBool::new(false),
            fault: AtomicU32::new(0),
            rmt_kicks: AtomicU32::new(0),
            rmt_busy: AtomicU32::new(0),
            rmt_poll_false: AtomicU32::new(0),
            rmt_refills: AtomicU32::new(0),
        }
    }

    pub fn state(&self) -> State {
        State::from_u32(self.state.load(Ordering::Relaxed)).unwrap_or(State::Error)
    }

    pub fn set_state(&self, s: State) {
        self.state.store(s as u32, Ordering::Relaxed);
    }
}

/// Global status (ARCHITECTURE §5.2): field-atomic, single writer per field,
/// read lock-free by the UI.
pub static STATUS: Status = Status::new();
