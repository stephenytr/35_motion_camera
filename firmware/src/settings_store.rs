//! Runtime settings/counter store (ARCHITECTURE §12) — the in-RAM shadow of
//! the persisted record. Tasks read/write it through a spinlock; the storage
//! task persists it at idle boundaries.
//!
//! Core-0-only by design (decision log #23/24): the supervisor, UI, and
//! storage tasks (all core 0) are the only users. The director (core 1)
//! deliberately never touches the store — esp-storage's flash writes
//! hardware-park core 1 mid-`with_store` critical section, and core 0 would
//! then spin on this lock forever (starved wdt_task → SysRtcWdt reboot).
//! Only tasks touch it, never ISRs, and the lock is never held across an
//! await — so the spin cannot deadlock (core-0 embassy tasks are
//! cooperative).
//!
//! Single-writer rule: any task may call `set` (it just marks dirty); only
//! the storage task clears the dirty flag.

use core::sync::atomic::{AtomicBool, Ordering};

use logic::settings::Settings;
use logic::storage_codec::Payload;

static LOCK: AtomicBool = AtomicBool::new(false);
static mut STORE: Payload = Payload {
    settings: Settings::defaults(),
    exposed_a: 0,
    exposed_b: 0,
};

static DIRTY: AtomicBool = AtomicBool::new(false);

#[cfg(feature = "debug-prints")]
static SPIN_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

fn with_store<T>(f: impl FnOnce(&mut Payload) -> T) -> T {
    while LOCK
        .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        #[cfg(feature = "debug-prints")]
        {
            let n = SPIN_COUNT.fetch_add(1, Ordering::Relaxed);
            if n % 1024 == 0 {
                log::warn!("dbg: store lock contended (spins {n})");
            }
        }
        core::hint::spin_loop();
    }
    // SAFETY: the spinlock above gives exclusive access; all users of the
    // static go through `with_store`.
    let r = f(unsafe { &mut *core::ptr::addr_of_mut!(STORE) });
    LOCK.store(false, Ordering::Release);
    r
}

pub fn get() -> Payload {
    with_store(|s| *s)
}

pub fn settings() -> Settings {
    get().settings
}

/// Update the settings; marks the store dirty for the storage task.
pub fn set_settings(s: Settings) {
    with_store(|slot| slot.settings = s);
    DIRTY.store(true, Ordering::SeqCst);
}

/// Update one track's exposed counter (0 = track A, 1 = track B).
pub fn set_exposed(track: u8, n: u32) {
    with_store(|slot| {
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
    with_store(|slot| *slot = p);
}

/// True if a write is pending; clears on read (storage task only).
pub fn dirty_take() -> bool {
    DIRTY.swap(false, Ordering::SeqCst)
}

/// True if a write is pending, without clearing (idle-gate check).
pub fn is_dirty() -> bool {
    DIRTY.load(Ordering::SeqCst)
}
