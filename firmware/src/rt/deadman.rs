//! Deadman (ARCHITECTURE §4.4): TIMG1 MWDT, two stages, P3 — the last line.
//!
//! Stage 0 fires an interrupt at the programmed timeout (2.5 × frame
//! period, min 100 ms, via `rt::arm_job`); the heartbeat *feeds* the WDT at
//! every firing. If the RT-plane chain stalls, stage 0 times out, this P3
//! ISR latches safe state, writes the fault marker, and disarms the WDT —
//! the now-unfed RTC watchdog reboots the chip, which boots into `ERROR WD`
//! via the marker.
//!
//! Stage 1 is the new hardware backstop (the classic ESP32 deadman had no
//! such thing — decision #34): it resets the system outright at 2× the
//! stage-0 timeout. If the interrupt path itself is dead (handler never
//! bound, interrupts masked, ISR corrupted), stage 1 still brings the chip
//! down instead of leaving the plane frozen with no watchdog at all.
//!
//! Register access is typed through the esp32s3 PAC: the classic code used
//! raw pointers at 0x3FF60000 with the old WDTCONFIG0/1/2 layout; the S3
//! TIMG1 lives at 0x60020000 with per-stage `WDTCONFIG[n].hold` registers
//! and `WDTCONFIG1` prescale (tick = 12.5 ns × prescale, so 80 → 1 µs).
//!
//! Lock discipline: this P3 ISR takes **no locks**. On Xtensa, critical
//! sections do not mask interrupts, so it can preempt a P2 ISR holding any
//! of the RT-plane mutexes — touching those mutexes here would self-deadlock
//! the core. It only touches raw registers, the lock-free SAFE latch, and
//! the lock-free fault atomics.

use esp_hal::interrupt::{InterruptHandler, Priority};
use esp_hal::peripherals::TIMG1;
use logic::interlock::ErrorCode;

use crate::fault;

/// Write-protection key: writing this value unlocks the WDT registers
/// (IDF `TIMG_WDT_WKEY_VALUE`, identical across ESP32 generations).
const WDT_UNLOCK_KEY: u32 = 0x50D8_3AA1;

/// MWDT clock prescale (WDTCONFIG1 [31:16]): tick = 12.5 ns × prescale.
/// 80 → tick = 1 µs, so `hold` is programmed in µs directly (32-bit range).
const WDT_PRESCALE: u32 = 80;

/// Stage actions (WDTCONFIG0 fields): 0 = off, 1 = interrupt, 2 = reset
/// CPU, 3 = reset system.
const STG_OFF: u8 = 0;
const STG_INTERRUPT: u8 = 1;
const STG_RESET_SYSTEM: u8 = 3;

/// Reset pulse lengths for the stage-1 reset (WDTCONFIG0): max value keeps
/// the chip held in reset long enough to fully discharge rails.
const RESET_LEN: u8 = 7;

fn unlock() {
    TIMG1::regs()
        .wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(WDT_UNLOCK_KEY) });
}

/// Bind the P3 ISR on core 1 and park the watchdog disarmed. Called in the
/// core-1 closure before the heartbeat.
pub fn init() {
    let tg = TIMG1::regs();
    unlock();
    tg.wdtconfig0().write(|w| unsafe { w.bits(0) }); // disarmed
    tg.int_clr().write(|w| w.wdt().bit(true));
    esp_hal::interrupt::bind_handler(
        esp_hal::peripherals::Interrupt::TG1_WDT_LEVEL,
        InterruptHandler::new(
            deadman_isr,
            Priority::Priority3, // ARCHITECTURE §2: last-line safety
        ),
    );
}

/// Program both stage timeouts (via `rt::arm_job`; ARCHITECTURE §4.4):
/// stage 0 fires the interrupt if the heartbeat does not feed within `us`
/// µs; stage 1 resets the system at 2× that if the interrupt path is dead.
pub fn set_timeout(us: u32) {
    let tg = TIMG1::regs();
    let us = us.max(1);
    unlock();
    tg.wdtconfig1()
        .write(|w| unsafe { w.wdt_clk_prescale().bits(WDT_PRESCALE as u16) });
    tg.wdtconfig(0).write(|w| unsafe { w.hold().bits(us) });
    tg.wdtconfig(1)
        .write(|w| unsafe { w.hold().bits(us.saturating_mul(2)) });
    tg.wdtconfig0().write(|w| unsafe {
        w.wdt_en().set_bit();
        w.wdt_flashboot_mod_en().clear_bit();
        w.wdt_stg0().bits(STG_INTERRUPT);
        w.wdt_stg1().bits(STG_RESET_SYSTEM);
        w.wdt_stg2().bits(STG_OFF);
        w.wdt_stg3().bits(STG_OFF);
        w.wdt_cpu_reset_length().bits(RESET_LEN);
        w.wdt_sys_reset_length().bits(RESET_LEN);
        w
    });
    feed();
}

/// Feed (heartbeat ISR, every firing): restart the whole watchdog cycle.
/// A single raw write — ISR-safe.
#[inline]
pub fn feed() {
    TIMG1::regs().wdtfeed().write(|w| unsafe { w.wdt_feed().bits(1) });
}

/// Disarm (heartbeat on park).
pub fn disarm() {
    let tg = TIMG1::regs();
    unlock();
    tg.wdtconfig0().write(|w| unsafe { w.bits(0) });
}

#[esp_hal::ram]
extern "C" fn deadman_isr() {
    let tg = TIMG1::regs();
    tg.int_clr().write(|w| w.wdt().bit(true));
    unlock();
    tg.wdtconfig0().write(|w| unsafe { w.bits(0) }); // disarm; no further IRQs
    crate::rt::latch_safe_state();
    fault::marker_write(ErrorCode::Watchdog.code());
    crate::fault::raise(ErrorCode::Watchdog);
}
