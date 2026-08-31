//! Shutter pair ISRs (ARCHITECTURE §3.1, §3.3): the two auxiliary one-shots
//! that finish the shutter waveform every frame.
//!
//! - Peak-hold (timg1.0, P2): fires 4 ms after pull-in, drops LEDC 100% → 25%.
//! - Exposure-end (timg0.1, P2): fires after exposure, drops LEDC 25% → 0%.
//!
//! Both are armed by the heartbeat ISR at ExposeStart; neither re-arms itself.
//! Handlers are bound on core 1 (setup inside the core-1 closure).
//!
//! Stale-ISR hazard: a one-shot that fired *just before* a door-open / JAM /
//! hard-fault event is still pending (or preempted mid-body) when that
//! event's `disarm()` + `safe_state()` run, and executes afterwards — with
//! only the `safe_active()` check it would re-energize the shutter while the
//! plane is halted (the door path deliberately does NOT latch safe state,
//! so that check cannot catch it). The `DISARMED` latch below closes the
//! window: `disarm()` and `safe_state()` set it, `arm_hold`/`arm_exposure`
//! clear it (the heartbeat is the only re-armer), and both ISRs refuse to
//! touch the driver while it is set.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

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

/// Disarmed latch (see module docs): starts set — no stray timer may drive
/// the shutter until the heartbeat's first arm.
static DISARMED: AtomicBool = AtomicBool::new(true);

/// Lock-free "driven off by safe state" latch (rt::safe_state calls this;
/// no CS mutexes, so it is callable pre-init and from any ISR). Inlined:
/// safe_state runs #[ram] while the flash cache can be off (flash-write
/// parking window), so this must not live in flash.
#[inline(always)]
pub fn latch_disarmed() {
    DISARMED.store(true, Ordering::SeqCst);
}

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
/// Re-arms the driver: a stale timer from a pre-safe-state frame must not
/// fire into a halted plane, but the *current* frame's pair may.
pub fn arm_hold(us: u32) {
    DISARMED.store(false, Ordering::SeqCst);
    HOLD.lock(|slot| {
        let mut slot = slot.borrow_mut();
        if let Some(t) = slot.as_mut() {
            let _ = t.schedule(Duration::from_micros(us as u64));
        }
    });
}

/// Arm the exposure-end one-shot (called by the heartbeat ISR at ExposeStart).
pub fn arm_exposure(us: u32) {
    DISARMED.store(false, Ordering::SeqCst);
    EXP_END.lock(|slot| {
        let mut slot = slot.borrow_mut();
        if let Some(t) = slot.as_mut() {
            let _ = t.schedule(Duration::from_micros(us as u64));
        }
    });
}

/// Stop both one-shots without waiting for them to fire. Used by the door
/// ISR (M2d) when forcing safe state outside the normal per-frame park path
/// — a queued hold/exposure-end firing after `safe_state()` would otherwise
/// re-energize the solenoid (see the stale-ISR hazard in the module docs).
pub fn disarm() {
    DISARMED.store(true, Ordering::SeqCst);
    HOLD.lock(|slot| {
        if let Some(t) = slot.borrow_mut().as_mut() {
            t.stop();
        }
    });
    EXP_END.lock(|slot| {
        if let Some(t) = slot.borrow_mut().as_mut() {
            t.stop();
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
    if crate::rt::safe_active() || DISARMED.load(Ordering::SeqCst) {
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
    if crate::rt::safe_active() || DISARMED.load(Ordering::SeqCst) {
        return;
    }
    shutter::off();
}
