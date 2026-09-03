//! RMT STEP generator (ARCHITECTURE §3.2) — one µstep per pulse period,
//! 480 µsteps (30 full steps) per frame, table preloaded from
//! `logic::profile`.
//!
//! The heartbeat ISR kicks one non-blocking transmit per FrameStart; the
//! table is 481 PulseCodes (480 periods + end marker) and fits the esp32's
//! full 512-word RMT RAM with `memsize = 8`, so no mid-transfer refill or
//! task participation is needed. The ISR reclaims the finished transaction
//! at the next frame start (a full 18+ ms after transfer end at 24 fps).
//!
//! Busy at kick time = the previous transfer overran the frame — per
//! ARCHITECTURE this publishes `Fault::RmtBusy` (caught one frame late by
//! design; the index watchdog is the backstop).

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::blocking_mutex::CriticalSectionMutex;
use esp_hal::gpio::interconnect::PeripheralOutput;
use esp_hal::gpio::Level;
use esp_hal::peripherals::RMT;
use esp_hal::rmt::{Channel, PulseCode, Rmt, Tx, TxChannelConfig, TxChannelCreator, TxTransaction};
use esp_hal::time::Rate;
use esp_hal::Blocking;
use logic::consts::FRAME_USTEPS;
use logic::profile::StepTable;

use crate::fault::ErrorCode;

/// 480 µstep periods + end marker.
const MAX_CODES: usize = FRAME_USTEPS as usize + 1;

/// Symbol table — written by the director while the RMT slot is idle,
/// borrowed ('static) by the in-flight transaction while busy.
static mut STEP_BUFFER: [PulseCode; MAX_CODES] = [PulseCode(0); MAX_CODES];
static STEP_LEN: AtomicU32 = AtomicU32::new(0);

enum Slot {
    Idle(Channel<'static, Blocking, Tx>),
    Busy(TxTransaction<'static, 'static>),
}

static SLOT: CriticalSectionMutex<RefCell<Option<Slot>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// Create the channel (called in the core-1 closure) and park it idle.
pub fn init(rmt: RMT<'static>, pin: impl PeripheralOutput<'static>) {
    let rmt = Rmt::new(rmt, Rate::from_mhz(80)).expect("rmt init");
    let channel = rmt
        .channel0
        .configure_tx(
            &TxChannelConfig::default()
                .with_clk_divider(80) // 80 MHz / 80 = 1 MHz, 1 tick = 1 µs
                .with_idle_output_level(Level::Low)
                .with_idle_output(true)
                .with_memsize(8), // whole 512-word RMT RAM
        )
        .expect("rmt tx config")
        .with_pin(pin);
    SLOT.lock(|slot| *slot.borrow_mut() = Some(Slot::Idle(channel)));
}

/// Rebuild the symbol table from a profile. Safe whenever the RMT channel
/// isn't physically mid-transfer — which is most of any frame period, since
/// the pulldown transfer (`pull_us`) is much shorter than the frame period
/// and a running job otherwise sits with a *finished* transaction un-
/// reclaimed until the next `kick()`. So, like `kick()`, this polls a
/// `Busy` slot and reclaims it if the transfer has actually completed.
/// Returns false only if a transfer is genuinely still in flight.
///
/// The buffer write happens *inside* the slot lock: `kick()` runs at P2 and
/// can preempt this task mid-write, and a kick that reads a half-updated
/// table would transmit one corrupt frame. Holding the lock for the write
/// masks interrupts for a few µs — acceptable, the pair timers are armed
/// separately and the heartbeat cadence is millisecond-scale.
pub fn build_table(table: &StepTable) -> bool {
    SLOT.lock(|slot| {
        let mut slot = slot.borrow_mut();
        let free = match slot.take() {
            None => true,
            Some(Slot::Idle(ch)) => {
                *slot = Some(Slot::Idle(ch));
                true
            }
            Some(Slot::Busy(mut tx)) => {
                if tx.poll() {
                    match tx.wait() {
                        Ok(ch) => {
                            *slot = Some(Slot::Idle(ch));
                            true
                        }
                        Err((_, ch)) => {
                            *slot = Some(Slot::Idle(ch));
                            true
                        }
                    }
                } else {
                    *slot = Some(Slot::Busy(tx));
                    false
                }
            }
        };
        if !free {
            crate::status::STATUS
                .rmt_poll_false
                .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            return false;
        }
        unsafe {
            for (i, &dt) in table.dt_us[..table.len].iter().enumerate() {
                let dt = dt.max(2);
                let hi = (dt / 2) as u16;
                let lo = (dt - hi as u32) as u16;
                STEP_BUFFER[i] = PulseCode::new(Level::High, hi, Level::Low, lo);
            }
            STEP_BUFFER[table.len] = PulseCode::end_marker();
        }
        STEP_LEN.store((table.len + 1) as u32, Ordering::Relaxed);
        true
    })
}

/// Start the next frame's transfer (heartbeat ISR at FrameStart).
pub fn kick() {
    crate::status::STATUS
        .rmt_kicks
        .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    SLOT.lock(|slot| {
        let mut slot = slot.borrow_mut();
        let cur = slot.take();
        let mut busy_raise = false;
        let next = match cur {
            None => None,
            Some(Slot::Idle(ch)) => start(ch),
            Some(Slot::Busy(mut tx)) => {
                if tx.poll() {
                    // Finished since the last frame start: reclaim (wait
                    // returns immediately) and start the new frame.
                    match tx.wait() {
                        Ok(ch) => start(ch),
                        Err((_, ch)) => {
                            crate::fault::raise(ErrorCode::RmtBusy);
                            busy_raise = true;
                            Some(Slot::Idle(ch))
                        }
                    }
                } else {
                    crate::fault::raise(ErrorCode::RmtBusy);
                    busy_raise = true;
                    Some(Slot::Busy(tx))
                }
            }
        };
        if busy_raise {
            // The previous transfer was genuinely still running at kick
            // time: RmtBusy raised. Count it for the wedge diagnostics.
            crate::status::STATUS
                .rmt_busy
                .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
        *slot = next;
    });
}

fn start(ch: Channel<'static, Blocking, Tx>) -> Option<Slot> {
    let len = STEP_LEN.load(Ordering::Relaxed) as usize;
    if len == 0 {
        return Some(Slot::Idle(ch)); // no table yet — stay parked
    }
    // SAFETY: STEP_BUFFER is only written while the slot is idle; the
    // transaction borrows it read-only for its (finite) duration.
    let data = unsafe { &STEP_BUFFER[..len] };
    match ch.transmit(data) {
        Ok(tx) => Some(Slot::Busy(tx)),
        Err((_, ch)) => {
            crate::fault::raise(ErrorCode::RmtBusy);
            Some(Slot::Idle(ch))
        }
    }
}
