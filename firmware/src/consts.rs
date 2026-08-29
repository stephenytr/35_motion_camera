//! SPECS/ARCHITECTURE constants plus firmware-only additions.

pub use logic::consts::*;

/// Xtensa interrupt priorities (esp_hal::interrupt::Priority) — ARCHITECTURE §2.
pub mod isr_priority {
    /// P3: deadman timer + door interlock (last-line safety).
    pub const SAFETY: u8 = 3;
    /// P2: heartbeat, exposure/peak-hold timers, index GPIO.
    pub const FRAME_TIMING: u8 = 2;
    /// P1: embassy executors and all async drivers (defaults).
    pub const ASYNC: u8 = 1;
}

/// HIL timing debug strobe pin (ARCHITECTURE §11): frame strobe + phase marker.
/// Bench: user LED on GPIO 13, active-high marks the frame phase.
pub const DEBUG_STROBE_GPIO: u8 = 13;
