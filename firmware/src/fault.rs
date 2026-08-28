//! Fault/event plumbing (ARCHITECTURE §5.2–5.3).

pub use logic::interlock::ErrorCode;

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
