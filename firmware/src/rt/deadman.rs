//! Deadman (ARCHITECTURE §4.4): TIMG1 watchdog stage 0 at P3 — the last line.
//!
//! The director programs the timeout per job (2.5 × frame period, min 100 ms,
//! via `rt::arm_job`); the heartbeat *feeds* the watchdog at every firing.
//! If the RT-plane chain stalls, stage 0 times out, this P3 ISR latches safe
//! state, writes the fault marker, and disarms the watchdog — the now-unfed
//! RTC watchdog reboots the chip, which boots into `ERROR WD` via the marker.
//!
//! Why the TIMG WDT: the esp-hal rc has no systimer driver for the esp32
//! (decision log #15's systimer alarm1 applies to the esp32-s3), and the
//! esp32's legacy FRC timers are ROM-owned and poorly documented. The TIMG
//! WDT is a real watchdog (hardware reset backstop available later) with an
//! identical register map on esp32 and esp32-s3. Registers below are from the
//! IDF `timer_group_reg.h`.
//!
//! Lock discipline: this P3 ISR takes **no locks**. On Xtensa, critical
//! sections do not mask interrupts, so it can preempt a P2 ISR holding any of
//! the RT-plane mutexes — touching those mutexes here would self-deadlock the
//! core. It only touches raw registers, the lock-free SAFE latch, and the
//! lock-free event channel.

use esp_hal::interrupt::{InterruptHandler, Priority};
use logic::interlock::ErrorCode;

use crate::fault::{self, Event, EVENTS};

// TIMG1 block (esp32: 0x3FF60000). Watchdog registers per IDF.
const TIMG1_BASE: u32 = 0x3FF6_0000;
const WDT_CONFIG0: *mut u32 = (TIMG1_BASE + 0x48) as *mut u32;
const WDT_CONFIG1: *mut u32 = (TIMG1_BASE + 0x4C) as *mut u32;
const WDT_CONFIG2: *mut u32 = (TIMG1_BASE + 0x50) as *mut u32;
const WDT_FEED: *mut u32 = (TIMG1_BASE + 0x60) as *mut u32;
const WDT_WPROTECT: *mut u32 = (TIMG1_BASE + 0x64) as *mut u32;
const INT_CLR: *mut u32 = (TIMG1_BASE + 0xA4) as *mut u32;

// WDTCONFIG0 bits (IDF TIMG_WDT_*).
const WDT_EN: u32 = 1 << 31;
const WDT_STG0_INT: u32 = 1 << 29; // stage 0 action = interrupt
const WDT_LEVEL_INT_EN: u32 = 1 << 21;
const WDT_CPU_RESET_LEN: u32 = 7 << 18;
const WDT_SYS_RESET_LEN: u32 = 7 << 15;
const WDT_INT_CLR_BIT: u32 = 1 << 2; // TIMG_INT_CLR_TIMERS WDT bit

/// Write-protection key: writing this value unlocks the WDT registers.
const WDT_UNLOCK_KEY: u32 = 0x50D8_3AA1;

/// SWDT clock prescale (WDTCONFIG1 [31:16]): tick = 12.5 ns × prescale.
/// 80 → tick = 1 µs, so STG0_HOLD is the timeout in µs (32-bit range).
const WDT_PRESCALE: u32 = 80;

/// Bind the P3 ISR on core 1 and park the watchdog disarmed. Called in the
/// core-1 closure before the heartbeat.
pub fn init() {
    unsafe {
        WDT_WPROTECT.write_volatile(WDT_UNLOCK_KEY);
        WDT_CONFIG0.write_volatile(0); // disarmed
        INT_CLR.write_volatile(WDT_INT_CLR_BIT);
    }
    esp_hal::interrupt::bind_handler(
        esp_hal::peripherals::Interrupt::TG1_WDT_LEVEL,
        InterruptHandler::new(
            deadman_isr,
            Priority::Priority3, // ARCHITECTURE §2: last-line safety
        ),
    );
}

/// Program the timeout (via `rt::arm_job`; ARCHITECTURE §4.4): the watchdog
/// fires if the heartbeat does not feed within `us` µs.
pub fn set_timeout(us: u32) {
    unsafe {
        WDT_WPROTECT.write_volatile(WDT_UNLOCK_KEY);
        WDT_CONFIG1.write_volatile(WDT_PRESCALE << 16);
        WDT_CONFIG2.write_volatile(us.max(1)); // STG0_HOLD, µs == ticks
        WDT_CONFIG0.write_volatile(
            WDT_EN | WDT_STG0_INT | WDT_LEVEL_INT_EN | WDT_CPU_RESET_LEN | WDT_SYS_RESET_LEN,
        );
        WDT_FEED.write_volatile(1);
    }
}

/// Feed (heartbeat ISR, every firing): restart the watchdog cycle. A single
/// raw write — ISR-safe.
#[inline]
pub fn feed() {
    unsafe { WDT_FEED.write_volatile(1) };
}

/// Disarm (heartbeat on park).
pub fn disarm() {
    unsafe {
        WDT_WPROTECT.write_volatile(WDT_UNLOCK_KEY);
        WDT_CONFIG0.write_volatile(0);
    }
}

#[esp_hal::ram]
extern "C" fn deadman_isr() {
    unsafe {
        INT_CLR.write_volatile(WDT_INT_CLR_BIT);
        WDT_WPROTECT.write_volatile(WDT_UNLOCK_KEY);
        WDT_CONFIG0.write_volatile(0); // disarm; no further IRQs
    }
    crate::rt::latch_safe_state();
    fault::marker_write(ErrorCode::Watchdog.code());
    let _ = EVENTS.try_send(Event::Fault(ErrorCode::Watchdog));
}
