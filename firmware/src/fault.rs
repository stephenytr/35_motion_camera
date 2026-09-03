//! Fault/event plumbing (ARCHITECTURE §5.2–5.3).

pub use logic::interlock::ErrorCode;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// Events from the RT plane / director to the command plane, delivered over an
/// embassy channel with `try_send` (never blocking from ISR context).
///
/// Rare events only (≤ ~1 Hz): high-rate frame counters bypass the channel
/// entirely and flow through `status::STATUS` atomics, which the supervisor
/// polls — the channel's critical-section mutex is per-core (decision log
/// #23), so cross-core sends are only sound at low rates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    JobComplete,
    DoorOpen,
    DoorClosed,
    CounterZero,
    SettingsChanged,
}

/// ISRs / director → tasks (ARCHITECTURE §5.2): multiple producers, bounded,
/// `try_send` only.
pub type EventChannel = Channel<CriticalSectionRawMutex, Event, 32>;

pub static EVENTS: EventChannel = EventChannel::new();

/// Raise a fault: write it to `status::STATUS.fault` directly. The
/// supervisor *polls* that atomic for deltas (its 20 ms loop) instead of
/// receiving an event — the channel's critical-section mutex is per-core
/// (decision log #23), and `RmtBusy` can raise at frame rate (~13 Hz) while
/// a transfer is stuck, which made cross-core `try_send`s from the core-1
/// ISR race the core-0 consumer at a rate the channel was never meant for.
/// ISR-safe: one atomic store, no locks.
pub fn raise(code: ErrorCode) {
    crate::status::STATUS
        .fault
        .store(code.code(), core::sync::atomic::Ordering::Relaxed);
}

/// Fault marker (ARCHITECTURE §4.4): a `.noinit` static that survives the
/// RTC-watchdog reboot, so the next boot can report what killed the last run.
/// Written by the deadman ISR before it stops re-arming; read (and cleared)
/// once at boot. The magic word guards against uninitialized RAM on cold
/// boots.
mod marker {
    const MARKER_MAGIC: u32 = 0xC0DE_FA1E;

    #[repr(C)]
    struct Marker {
        magic: u32,
        code: u32,
    }

    #[link_section = ".noinit"]
    static mut FAULT_MARKER: Marker = Marker { magic: 0, code: 0 };

    /// Write the fault code. ISR-safe: plain volatile RAM writes.
    pub fn write(code: u32) {
        unsafe {
            let p = core::ptr::addr_of_mut!(FAULT_MARKER);
            (*p).magic = MARKER_MAGIC;
            (*p).code = code;
        }
    }

    /// Boot check: `Some(code)` if a fault survived the reset. Clears the
    /// marker so the boot that follows a *normal* run reports nothing.
    pub fn take() -> Option<u32> {
        unsafe {
            let p = core::ptr::addr_of_mut!(FAULT_MARKER);
            let m = core::ptr::read_volatile(p);
            if m.magic == MARKER_MAGIC {
                (*p).magic = 0;
                (*p).code = 0;
                Some(m.code)
            } else {
                None
            }
        }
    }
}

/// Boot-time check for a marker left by the last run (see `marker`).
pub fn marker_take() -> Option<u32> {
    marker::take()
}

/// Record a fault so the next boot can report it (deadman ISR only today).
pub fn marker_write(code: u32) {
    marker::write(code);
}
