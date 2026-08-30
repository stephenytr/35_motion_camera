//! Settings/counter persistence (ARCHITECTURE §12): idle-gated flash writes
//! only (run stop, door events, roll end, settings change), ping-pong 4 KB
//! sectors, versioned `logic::storage_codec` records + CRC32.
//!
//! Media is behind `StorageBackend`. The bench backend is two RAM sectors —
//! honest for the codec/policy path but not non-volatile (a reboot starts at
//! "fresh flash"). The real flash driver (esp-hal FlashStorage on a data
//! partition) lands once the partition layout is confirmed; the policy
//! loop and codec here are exactly what it will sit on.

use embassy_time::{Duration, Ticker};
use log::info;

use crate::fault::{Event, EVENTS};
use crate::settings_store;
use crate::status::{State, Status};
use logic::storage_codec::{self, Slot, SECTOR_SIZE};

/// Physical media abstraction (see module docs).
pub trait StorageBackend {
    /// Read both sectors (A first).
    fn read_sectors(&mut self) -> ([u8; SECTOR_SIZE], [u8; SECTOR_SIZE]);
    /// Erase-and-write one sector with the encoded record.
    fn write_sector(&mut self, slot: Slot, sector: &[u8; SECTOR_SIZE]);
}

/// Bench backend: RAM ping-pong sectors. Not non-volatile by design.
struct RamBackend {
    a: [u8; SECTOR_SIZE],
    b: [u8; SECTOR_SIZE],
}

impl RamBackend {
    fn new() -> Self {
        Self {
            a: [0xFF; SECTOR_SIZE],
            b: [0xFF; SECTOR_SIZE],
        }
    }
}

impl StorageBackend for RamBackend {
    fn read_sectors(&mut self) -> ([u8; SECTOR_SIZE], [u8; SECTOR_SIZE]) {
        (self.a, self.b)
    }

    fn write_sector(&mut self, slot: Slot, sector: &[u8; SECTOR_SIZE]) {
        match slot {
            Slot::A => self.a = *sector,
            Slot::B => self.b = *sector,
        }
    }
}

#[embassy_executor::task]
pub async fn storage_task(status: &'static Status) {
    info!("storage: up, idle-gated persistence checker at 1 Hz");
    let mut backend = RamBackend::new();
    let mut ticker = Ticker::every(Duration::from_secs(1));

    // Boot load: newest valid record wins, else defaults (fresh flash).
    let (a, b) = backend.read_sectors();
    let mut seq = 0u32;
    match storage_codec::choose(&a, &b) {
        Some((slot, s, payload)) => {
            info!(
                "storage: loaded record seq {s} from sector {slot:?} (fps={}, roll={})",
                payload.settings.fps, payload.settings.roll_frames
            );
            settings_store::load(payload);
            seq = s;
        }
        None => {
            info!("storage: fresh flash — defaults active");
        }
    }

    loop {
        ticker.next().await;
        if !settings_store::is_dirty() {
            continue;
        }
        match status.state() {
            State::Idle | State::Door | State::Error => {
                // ARCHITECTURE §12: flash writes only at idle boundaries.
                let payload = settings_store::get();
                seq = seq.wrapping_add(1);
                let (a, b) = backend.read_sectors();
                let live = storage_codec::choose(&a, &b).map(|(slot, _, _)| slot);
                let next_slot = match live {
                    Some(slot) => slot.other(),
                    None => Slot::B, // first write ever goes to B
                };
                let mut sector = [0xFFu8; SECTOR_SIZE];
                storage_codec::encode(&mut sector, seq, &payload);
                backend.write_sector(next_slot, &sector);
                settings_store::dirty_take();
                info!("storage: persisted seq {seq} to sector {next_slot:?}");
                let _ = EVENTS.try_send(Event::SettingsChanged);
            }
            _ => {
                // Transport active: keep the dirty flag, retry at the next
                // idle boundary (invariant 4, ARCHITECTURE §12).
            }
        }
    }
}
