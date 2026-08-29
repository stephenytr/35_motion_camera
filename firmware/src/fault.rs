//! Fault/event plumbing (ARCHITECTURE §5.2–5.3).

pub use logic::interlock::ErrorCode;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// Events from the RT plane / director to the command plane, delivered over an
/// embassy channel with `try_send` (never blocking from ISR context).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    FrameDone(u32),
    JobComplete,
    IndexTick,
    DoorOpen,
    DoorClosed,
    Fault(ErrorCode),
    CounterZero,
    SettingsChanged,
}

/// ISRs / director → tasks (ARCHITECTURE §5.2): multiple producers, bounded,
/// `try_send` only.
pub type EventChannel = Channel<CriticalSectionRawMutex, Event, 32>;

pub static EVENTS: EventChannel = EventChannel::new();
