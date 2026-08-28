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
    pub state: AtomicU32,
    pub door_open: AtomicBool,
    pub vbat_mv: AtomicU32,
    /// 0 = none, else ErrorCode discriminant + payload (ARCHITECTURE §5.3).
    pub fault: AtomicU32,
}

impl Status {
    pub const fn new() -> Self {
        Self {
            frames_exposed: AtomicU32::new(0),
            state: AtomicU32::new(State::Idle as u32),
            door_open: AtomicBool::new(false),
            vbat_mv: AtomicU32::new(0),
            fault: AtomicU32::new(0),
        }
    }

    pub fn state(&self) -> State {
        State::from_u32(self.state.load(Ordering::Relaxed)).unwrap_or(State::Error)
    }

    pub fn set_state(&self, s: State) {
        self.state.store(s as u32, Ordering::Relaxed);
    }
}
