//! Settings/counter persistence (ARCHITECTURE §12): idle-gated flash writes
//! only (run stop, door events, roll end, settings change), ping-pong 4 KB
//! sectors, versioned `logic::storage_codec` records + CRC32.
//!
//! Media: the first two 4 KB sectors of the `nvs` partition region
//! (flash 0x9000/0xA000 — the IDF boot log confirms a 24 KB `nvs` partition
//! at 0x9000 and nothing else in the firmware uses it). Backend is
//! `esp-storage` `FlashStorage` with `multicore_auto_park`: writes park
//! core 1 for their duration (erase + program, ~100-200 ms). Writes only
//! happen at idle boundaries (§12), when the RT plane has no cadence to
//! keep — the only deferred core-1 work is a possible door ISR, delayed by
//! the write duration: the known flash-on-the-same-bus tradeoff (decision
//! #13). RT-plane code is `#[ram]`, so the flash-cache stall is inert.

use embassy_time::{Duration, Ticker};
use log::{info, warn};

use crate::fault::{Event, EVENTS};
use crate::settings_store;
use crate::status::Status;
use logic::storage_codec::{self, Slot, SECTOR_SIZE};

/// Physical media abstraction (see module docs).
pub trait StorageBackend {
    /// Read both sectors (A first).
    fn read_sectors(&mut self) -> ([u8; SECTOR_SIZE], [u8; SECTOR_SIZE]);
    /// Erase-and-write one sector with the encoded record; false on error.
    fn write_sector(&mut self, slot: Slot, sector: &[u8; SECTOR_SIZE]) -> bool;
}

/// Real flash backend on the `nvs` partition region (see module docs).
struct FlashBackend {
    flash: esp_storage::FlashStorage<'static>,
}

impl FlashBackend {
    const SECTOR_A: u32 = 0x9000;
    const SECTOR_B: u32 = 0x9000 + SECTOR_SIZE as u32;

    fn new() -> Self {
        // Panics if constructed twice: this is the single flash owner.
        // SAFETY: `FLASH` is not exposed through `esp_hal::init()`
        // peripherals; `steal()` is the esp-storage-sanctioned path. This
        // is the only flash owner in the firmware; reentrancy is guarded
        // by esp-storage's internal critical-section mutex, and cross-core
        // safety comes from `multicore_auto_park` below.
        let flash = unsafe { esp_storage::Flash::steal() };
        let flash = esp_storage::FlashStorage::new(flash).multicore_auto_park();
        Self { flash }
    }

    fn sector_addr(slot: Slot) -> u32 {
        match slot {
            Slot::A => Self::SECTOR_A,
            Slot::B => Self::SECTOR_B,
        }
    }
}

impl StorageBackend for FlashBackend {
    fn read_sectors(&mut self) -> ([u8; SECTOR_SIZE], [u8; SECTOR_SIZE]) {
        let mut a = [0xFFu8; SECTOR_SIZE];
        let mut b = [0xFFu8; SECTOR_SIZE];
        if self.flash.read_nor(Self::SECTOR_A, &mut a).is_err() {
            warn!("storage: flash read sector A failed");
        }
        if self.flash.read_nor(Self::SECTOR_B, &mut b).is_err() {
            warn!("storage: flash read sector B failed");
        }
        (a, b)
    }

    fn write_sector(&mut self, slot: Slot, sector: &[u8; SECTOR_SIZE]) -> bool {
        let addr = Self::sector_addr(slot);
        // One 4 KB sector erase, then program. Both park core 1 briefly.
        #[cfg(feature = "debug-prints")]
        let t0 = esp_hal::time::Instant::now();
        match self.flash.erase(addr, addr + SECTOR_SIZE as u32) {
            Ok(()) => {
                #[cfg(feature = "debug-prints")]
                log::info!(
                    "storage: erase done ({} ms)",
                    (esp_hal::time::Instant::now() - t0).as_millis()
                );
                match self.flash.write_nor(addr, sector) {
                    Ok(()) => {
                        #[cfg(feature = "debug-prints")]
                        log::info!(
                            "storage: program done ({} ms total)",
                            (esp_hal::time::Instant::now() - t0).as_millis()
                        );
                        true
                    }
                    Err(e) => {
                        warn!("storage: flash write {slot:?} failed: {e:?}");
                        false
                    }
                }
            }
            Err(e) => {
                warn!("storage: flash erase {slot:?} failed: {e:?}");
                false
            }
        }
    }
}

#[embassy_executor::task]
pub async fn storage_task(_status: &'static Status) {
    info!("storage: up, idle-gated persistence checker at 1 Hz (flash @ 0x9000)");
    let mut backend = FlashBackend::new();
    let mut ticker = Ticker::every(Duration::from_secs(1));

    // Boot load: newest valid record wins, else defaults (fresh flash).
    let (a, b) = backend.read_sectors();
    let mut seq = 0u32;
    match storage_codec::choose(&a, &b) {
        Some((slot, s, payload)) => {
            info!(
                "storage: loaded record seq {s} from sector {slot:?} (fps={}, roll={}, exposed_a={})",
                payload.settings.fps, payload.settings.roll_frames, payload.exposed_a
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
        crate::liveness::LIVENESS.bump_storage();
        #[cfg(feature = "debug-prints")]
        log::info!("dbg: storage tick dirty={}", settings_store::is_dirty());
        if !settings_store::is_dirty() {
            continue;
        }
        // ARCHITECTURE §12: flash writes only at idle boundaries. Gate on
        // the RT plane directly (`heartbeat::is_parked()`), not on
        // `status.state()` — nothing ever set `State::Run` (RunToggle only
        // sets Idle/Door/Error/Single/Inch), so the old match let a flash
        // write land *while the transport was actively running*. The
        // erase+program parks core 1 for ~100-200 ms; the heartbeat ISR
        // isn't `#[ram]`-resident, so it stalls until flash is readable
        // again, misses its deadman feed, and the (RAM-resident) deadman
        // fires — latching safe state and, after the RTC watchdog stops
        // being fed, rebooting the chip. That's the "motor stops, counters
        // reset to 0" bug: a full unplanned reboot mid-take.
        if crate::rt::heartbeat::is_parked() {
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
            // Log before the write: erase+program parks core 1, and if the
            // park ever wedges (see settings_store doc), this line is the
            // last thing on serial before the SysRtcWdt reboot.
            info!("storage: writing seq {seq} to sector {next_slot:?} (core 1 parked)");
            if backend.write_sector(next_slot, &sector) {
                settings_store::dirty_take();
                info!("storage: persisted seq {seq} to sector {next_slot:?}");
                let _ = EVENTS.try_send(Event::SettingsChanged);
            } else {
                warn!("storage: persist failed — retrying next idle tick");
            }
        }
        // Transport active: keep the dirty flag, retry at the next idle
        // boundary (invariant 4, ARCHITECTURE §12).
    }
}
