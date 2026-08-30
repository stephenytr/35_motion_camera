//! Door interlock ISR (GPIO 4, P3, core 1) — ARCHITECTURE §4.5.
//!
//! SPECS §6.1 fail-safe wiring: closed = switch pulls the line low; open (or
//! a broken/disconnected wire) = high. Configured with an internal pull-up so
//! a disconnected bench pin reads "open" by default, matching that fail-safe
//! without needing hardware.
//!
//! Debounce deviates from ARCHITECTURE's literal "re-arm a one-shot" wording:
//! all 4 TIMG general timers are already owned (heartbeat, shutter hold,
//! shutter exposure, esp_rtos scheduler; decision log #15/#17), so there is
//! no 5th timer to dedicate here. Instead this debounces by timestamp: an
//! edge within the debounce window of the last *accepted* edge is bounce and
//! is ignored. `esp_hal::time::Instant::now()` on esp32 is backed by TIMG0's
//! free-running LACT counter — a hardware block distinct from the T0/T1
//! GPTimers used elsewhere — so reading it here carries no ownership
//! conflict and needs no extra resource. Still P3, still no task
//! involvement, still fires the immediate safe-state action on the accepted
//! edge.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::CriticalSectionMutex;
use esp_hal::gpio::{Event as GpioEvent, Input, InputConfig, Io, Pull};
use esp_hal::interrupt::{InterruptHandler, Priority};
use esp_hal::peripherals::{GPIO4, IO_MUX};
use esp_hal::time::{Duration, Instant};

use crate::fault::{Event, EVENTS};

/// Bounce window (ARCHITECTURE §4.5).
const DEBOUNCE: Duration = Duration::from_millis(20);

static DOOR: CriticalSectionMutex<RefCell<Option<Input<'static>>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// Timestamp (µs since boot, truncated to 32 bits) of the last *accepted*
/// edge. Xtensa esp32 has no native 64-bit atomics; truncated + wrapping
/// arithmetic is exact for a 20 ms window (wraps every ~71 min).
static LAST_EDGE_US: AtomicU32 = AtomicU32::new(0);

/// Current debounced state. Defaults `true` (open) so a read before `init`
/// fails safe.
static DOOR_OPEN: AtomicBool = AtomicBool::new(true);

/// Bind the ISR on core 1 (GPIO interrupts are shared across pins — `Io`
/// installs the one dispatcher; `Input::listen` selects the edge per pin).
pub fn init(io_mux: IO_MUX<'static>, pin: GPIO4<'static>) {
    let mut io = Io::new(io_mux);
    io.set_interrupt_handler(InterruptHandler::new(
        door_isr,
        Priority::Priority3, // ARCHITECTURE §2: last-line safety
    ));

    let mut input = Input::new(pin, InputConfig::default().with_pull(Pull::Up));
    DOOR_OPEN.store(input.is_high(), Ordering::SeqCst);
    LAST_EDGE_US.store(
        Instant::now().duration_since_epoch().as_micros() as u32,
        Ordering::SeqCst,
    );
    input.listen(GpioEvent::AnyEdge);

    DOOR.lock(|slot| *slot.borrow_mut() = Some(input));
}

/// Debounced door state (supervisor / self-test read this at boot).
pub fn is_open() -> bool {
    DOOR_OPEN.load(Ordering::SeqCst)
}

extern "C" fn door_isr() {
    DOOR.lock(|slot| {
        let mut slot = slot.borrow_mut();
        let Some(pin) = slot.as_mut() else {
            return;
        };
        if !pin.is_interrupt_set() {
            return; // some other GPIO pin's edge
        }
        pin.clear_interrupt();

        let now_us = Instant::now().duration_since_epoch().as_micros() as u32;
        let last_us = LAST_EDGE_US.load(Ordering::Relaxed);
        if now_us.wrapping_sub(last_us) < DEBOUNCE.as_micros() as u32 {
            return; // bounce: within the window of the last accepted edge
        }
        LAST_EDGE_US.store(now_us, Ordering::Relaxed);

        let open = pin.is_high();
        apply_state(open);
    });
}

/// The action side of an accepted edge — shared by the real ISR and the TEMP
/// bench hook below, so both exercise the identical safe-state/event path.
fn apply_state(open: bool) {
    if open == DOOR_OPEN.swap(open, Ordering::SeqCst) {
        return; // level unchanged (both edges landed on the same side)
    }

    if open {
        // Immediate safe state (ARCHITECTURE §4.5) — recoverable, so this
        // does *not* go through `latch_safe_state`: that also stops the
        // RTC-WDT feed and forces a reboot, which is the deadman's
        // escalation path, not a normal door event.
        crate::rt::safe_state();
        crate::rt::heartbeat::halt();
        crate::rt::shutter::disarm();
        crate::rt::deadman::disarm();
        let _ = EVENTS.try_send(Event::DoorOpen);
    } else {
        let _ = EVENTS.try_send(Event::DoorClosed);
    }
}

/// TEMP M2d bench hook: no door switch is wired yet, so exercise the
/// safe-state/event path directly instead of the GPIO edge + debounce (that
/// part is standard `esp-hal` input handling, not custom logic). Removed
/// once real hardware is on the bench.
#[allow(dead_code)]
pub fn debug_force(open: bool) {
    apply_state(open);
}
