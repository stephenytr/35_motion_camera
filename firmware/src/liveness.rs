//! Command-plane task liveness (SPECS §8.2/§11 "task WDT").
//!
//! `wdt.rs`'s RTC-watchdog feed was previously gated only on
//! `rt::safe_active()` — a real RT-plane hang (frame timing) is caught, but
//! a command-plane task that hangs while still *yielding* (e.g. spins on an
//! await without making forward progress) is invisible to that check: the
//! feed loop keeps running and keeps feeding forever. Each command-plane
//! task bumps its own counter once per loop iteration; `wdt_task` only feeds
//! if every counter has advanced within its check window, so a stuck-but-
//! yielding task is also caught, not just a fully blocking one.

use core::sync::atomic::{AtomicU32, Ordering};

pub struct Liveness {
    supervisor: AtomicU32,
    ui: AtomicU32,
    director: AtomicU32,
    power: AtomicU32,
    storage: AtomicU32,
}

/// One loop-iteration snapshot of every task's counter.
pub type Snapshot = [u32; 5];

impl Liveness {
    pub const fn new() -> Self {
        Self {
            supervisor: AtomicU32::new(0),
            ui: AtomicU32::new(0),
            director: AtomicU32::new(0),
            power: AtomicU32::new(0),
            storage: AtomicU32::new(0),
        }
    }

    pub fn bump_supervisor(&self) {
        self.supervisor.fetch_add(1, Ordering::Relaxed);
    }
    pub fn bump_ui(&self) {
        self.ui.fetch_add(1, Ordering::Relaxed);
    }
    pub fn bump_director(&self) {
        self.director.fetch_add(1, Ordering::Relaxed);
    }
    pub fn bump_power(&self) {
        self.power.fetch_add(1, Ordering::Relaxed);
    }
    pub fn bump_storage(&self) {
        self.storage.fetch_add(1, Ordering::Relaxed);
    }

    pub fn snapshot(&self) -> Snapshot {
        [
            self.supervisor.load(Ordering::Relaxed),
            self.ui.load(Ordering::Relaxed),
            self.director.load(Ordering::Relaxed),
            self.power.load(Ordering::Relaxed),
            self.storage.load(Ordering::Relaxed),
        ]
    }
}

pub static LIVENESS: Liveness = Liveness::new();
