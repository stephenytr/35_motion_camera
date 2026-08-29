//! Shutter pair ISRs (ARCHITECTURE §3.1, §3.3): the two auxiliary one-shots
//! that finish the shutter waveform every frame.
//!
//! - Peak-hold (timg1.0, P2): fires 4 ms after pull-in, drops LEDC 100% → 25%.
//! - Exposure-end (timg0.1, P2): fires after exposure, drops LEDC 25% → 0%.
//!
//! Both are armed by the heartbeat ISR at ExposeStart; neither re-arms itself.
//! Handlers are bound on core 1 (setup inside the core-1 closure).

use core::cell::RefCell;

use embassy_sync::blocking_mutex::CriticalSectionMutex;
use esp_hal::interrupt::{InterruptHandler, Priority};
use esp_hal::time::Duration;
use esp_hal::timer::{timg::Timer, OneShotTimer};
use esp_hal::Blocking;

use crate::drivers::shutter;

static HOLD: CriticalSectionMutex<RefCell<Option<OneShotTimer<'static, Blocking>>>> =
    CriticalSectionMutex::new(RefCell::new(None));
static EXP_END: CriticalSectionMutex<RefCell<Option<OneShotTimer<'static, Blocking>>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// Bind both ISRs on core 1.
pub fn init(hold_timer: Timer<'static>, exp_timer: Timer<'static>) {
    let mut hold = OneShotTimer::new(hold_timer);
    hold.set_interrupt_handler(InterruptHandler::new(
        hold_isr,
        Priority::Priority2, // ARCHITECTURE §2: frame timing at P2
    ));
    hold.listen();

    let mut exp = OneShotTimer::new(exp_timer);
    exp.set_interrupt_handler(InterruptHandler::new(
        exp_end_isr,
        Priority::Priority2,
    ));
    exp.listen();

    HOLD.lock(|slot| *slot.borrow_mut() = Some(hold));
    EXP_END.lock(|slot| *slot.borrow_mut() = Some(exp));
}

/// Arm the peak-hold one-shot (called by the heartbeat ISR at ExposeStart).
pub fn arm_hold(us: u32) {
    HOLD.lock(|slot| {
        let mut slot = slot.borrow_mut();
        if let Some(t) = slot.as_mut() {
            let _ = t.schedule(Duration::from_micros(us as u64));
        }
    });
}

/// Arm the exposure-end one-shot (called by the heartbeat ISR at ExposeStart).
pub fn arm_exposure(us: u32) {
    EXP_END.lock(|slot| {
        let mut slot = slot.borrow_mut();
        if let Some(t) = slot.as_mut() {
            let _ = t.schedule(Duration::from_micros(us as u64));
        }
    });
}

extern "C" fn hold_isr() {
    HOLD.lock(|slot| {
        let mut slot = slot.borrow_mut();
        let Some(t) = slot.as_mut() else {
            return;
        };
        t.clear_interrupt();
    });
    if crate::rt::safe_active() {
        return;
    }
    shutter::hold();
}

extern "C" fn exp_end_isr() {
    EXP_END.lock(|slot| {
        let mut slot = slot.borrow_mut();
        let Some(t) = slot.as_mut() else {
            return;
        };
        t.clear_interrupt();
    });
    if crate::rt::safe_active() {
        return;
    }
    shutter::off();
}
