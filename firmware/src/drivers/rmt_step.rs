//! RMT STEP generator (ARCHITECTURE §3.2) — one µstep per pulse period,
//! 480 µsteps (30 full steps) per frame, table preloaded from
//! `logic::profile`.
//!
//! The heartbeat ISR kicks one non-blocking transmit per FrameStart; the
//! table is 481 PulseCodes (480 periods + end marker).
//!
//! Memory model changed with the ESP32-S3 (decision #34/#36): the classic
//! chip's 512-word RMT RAM held the whole table in one load; the S3 has
//! only 192 words (4 × 48), so the table no longer fits. The transfer now
//! streams: esp-hal preloads the first 192 entries and a dedicated P2 RMT
//! interrupt services the hardware's threshold events, refilling the FIFO
//! from the prebuilt table while the tail is still playing. The threshold
//! sits at half the channel RAM (96 words); worst-case drain rate at 36 fps
//! is ~31 entries/ms, so the ~3 ms runway has an interrupt-latency margin
//! of several orders of magnitude — refill cannot miss unless interrupts
//! are dead, in which case the RMT underrun raises the usual RmtBusy fault.
//!
//! Busy at kick time = the previous transfer overran the frame — per
//! ARCHITECTURE this publishes `Fault::RmtBusy` (caught one frame late by
//! design; the index watchdog is the backstop).

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::blocking_mutex::CriticalSectionMutex;
use esp_hal::gpio::interconnect::PeripheralOutput;
use esp_hal::gpio::Level;
use esp_hal::interrupt::{InterruptHandler, Priority};
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
/// Binds the S3 mid-transfer refill ISR (P2, core 1) on the RMT threshold
/// interrupt — see the module docs.
pub fn init(rmt: RMT<'static>, pin: impl PeripheralOutput<'static>) {
    let rmt = Rmt::new(rmt, Rate::from_mhz(80)).expect("rmt init");
    let channel = rmt
        .channel0
        .configure_tx(
            &TxChannelConfig::default()
                .with_clk_divider(80) // 80 MHz / 80 = 1 MHz, 1 tick = 1 µs
                .with_idle_output_level(Level::Low)
                .with_idle_output(true)
                .with_memsize(4), // S3: 4 × 48 words = the whole 192-word RAM
        )
        .expect("rmt tx config")
        .with_pin(pin);
    SLOT.lock(|slot| *slot.borrow_mut() = Some(Slot::Idle(channel)));

    // Refill plumbing: fire the P2 ISR whenever channel 0 drains to the
    // threshold watermark. esp-hal's blocking driver polls the same status
    // bits, so the ISR just borrows the transaction and calls `poll()`.
    let regs = esp_hal::peripherals::RMT::regs();
    regs.int_ena()
        .modify(|_, w| w.ch_tx_thr_event(0).bit(true));
    esp_hal::interrupt::bind_handler(
        esp_hal::peripherals::Interrupt::RMT,
        InterruptHandler::new(rmt_refill_isr, Priority::Priority2),
    );
}

/// P2 mid-transfer refill (S3, decision #36): the hardware raised the TX
/// threshold event — top the channel RAM back up from the prebuilt table.
/// Lock discipline: SLOT's critical-section mutex is per-core (core 1), and
/// the heartbeat ISR (also P2) cannot nest us; the director's build_table
/// and the heartbeat's kick() contend for the same lock on this core, so
/// the critical section is short (a handful of register writes).
#[esp_hal::ram]
extern "C" fn rmt_refill_isr() {
    // Edge-triggered clear first so the event can re-fire.
    esp_hal::peripherals::RMT::regs()
        .int_clr()
        .write(|w| w.ch_tx_thr_event(0).bit(true));

    crate::status::STATUS
        .rmt_refills
        .fetch_add(1, core::sync::atomic::Ordering::Relaxed);

    SLOT.lock(|slot| {
        let mut slot = slot.borrow_mut();
        match slot.take() {
            Some(Slot::Busy(mut tx)) => {
                // poll() clears the status bit and, on a threshold event,
                // writes the next chunk of the table into the freed RAM.
                // If it returns true the transfer finished — leave it in
                // the slot; the next kick() reclaims it (existing design).
                tx.poll();
                *slot = Some(Slot::Busy(tx));
            }
            // Idle/None: spurious event (e.g. the transfer ended between
            // the hardware event and this ISR). Nothing to do.
            Some(Slot::Idle(ch)) => *slot = Some(Slot::Idle(ch)),
            None => {}
        }
    });
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
