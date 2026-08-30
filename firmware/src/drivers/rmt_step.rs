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

use crate::fault::{ErrorCode, Event, EVENTS};

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

/// Rebuild the symbol table from a profile. Only safe while the RMT slot is
/// idle (the director does this at arm time, between jobs). Returns false
/// if a transfer is still in flight.
pub fn build_table(table: &StepTable) -> bool {
    let ok = SLOT.lock(|slot| matches!(slot.borrow().as_ref(), None | Some(Slot::Idle(_))));
    if !ok {
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
}

/// Start the next frame's transfer (heartbeat ISR at FrameStart).
pub fn kick() {
    SLOT.lock(|slot| {
        let mut slot = slot.borrow_mut();
        let cur = slot.take();
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
                            let _ = EVENTS.try_send(Event::Fault(ErrorCode::RmtBusy));
                            Some(Slot::Idle(ch))
                        }
                    }
                } else {
                    let _ = EVENTS.try_send(Event::Fault(ErrorCode::RmtBusy));
                    Some(Slot::Busy(tx))
                }
            }
        };
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
            let _ = EVENTS.try_send(Event::Fault(ErrorCode::RmtBusy));
            Some(Slot::Idle(ch))
        }
    }
}
