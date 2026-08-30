//! Runtime settings/counter store (ARCHITECTURE §12) — the in-RAM shadow of
//! the persisted record. Tasks read/write it through a critical-section
//! mutex; the storage task persists it at idle boundaries.
//!
//! Single-writer rule: any task may call `set` (it just marks dirty); only
//! the storage task clears the dirty flag.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::CriticalSectionMutex;
use logic::settings::Settings;
use logic::storage_codec::Payload;

static STORE: CriticalSectionMutex<RefCell<Payload>> =
    CriticalSectionMutex::new(RefCell::new(Payload {
        settings: Settings::defaults(),
        exposed_a: 0,
        exposed_b: 0,
    }));

static DIRTY: AtomicBool = AtomicBool::new(false);

pub fn get() -> Payload {
    STORE.lock(|s| *s.borrow())
}

pub fn settings() -> Settings {
    get().settings
}

/// Update the settings; marks the store dirty for the storage task.
pub fn set_settings(s: Settings) {
    STORE.lock(|slot| {
        slot.borrow_mut().settings = s;
    });
    DIRTY.store(true, Ordering::SeqCst);
}

/// Update one track's exposed counter (0 = track A, 1 = track B). Unused
/// until the counters wiring (UI/menu milestone) lands — see logic::counters.
#[allow(dead_code)]
pub fn set_exposed(track: u8, n: u32) {
    STORE.lock(|slot| {
        let mut slot = slot.borrow_mut();
        if track == 0 {
            slot.exposed_a = n;
        } else {
            slot.exposed_b = n;
        }
    });
    DIRTY.store(true, Ordering::SeqCst);
}

/// Replace the whole record (storage task at boot after flash load).
pub fn load(p: Payload) {
    STORE.lock(|slot| *slot.borrow_mut() = p);
}

/// True if a write is pending; clears on read (storage task only).
pub fn dirty_take() -> bool {
    DIRTY.swap(false, Ordering::SeqCst)
}

/// True if a write is pending, without clearing (idle-gate check).
pub fn is_dirty() -> bool {
    DIRTY.load(Ordering::SeqCst)
}
