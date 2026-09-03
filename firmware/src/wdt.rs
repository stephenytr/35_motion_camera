//! RTC-watchdog feeder (ARCHITECTURE §4.4, §5.1; SPECS §8.2 "task WDT"):
//! feeds the RTC watchdog every 100 ms while the system is healthy. When
//! the deadman (or door) ISR latches safe state, this task sees the latch,
//! stops feeding, and lets the watchdog reboot the chip — the deadman's
//! fault marker is then read at boot.
//!
//! Feeding is gated on two independent checks: `rt::safe_active()` catches
//! an RT-plane (frame-timing) hang, and the `liveness` module's per-task
//! counters catch a command-plane task that hangs while still *yielding*
//! (still `await`ing, making no counter progress) — a plain time-based feed
//! would keep servicing the watchdog forever in that case, making it
//! useless for anything but a fully blocking hang.
//!
//! The timeout is 5 s (was 2 s): flash persistence writes run with core-0
//! interrupts off and core 1 parked — an erase+program can legitimately
//! take a couple of seconds on a slow part, and 2 s turned a *healthy*
//! write into a reboot. 5 s still catches a truly wedged system quickly
//! enough that the deadman fault marker tells us why at next boot.
//!
//! Note `esp_hal::init` disables the RWDT at boot; this task re-arms it. The
//! `Rwdt` handle is the ZST field of `Rtc`, which takes the RTC_TIMER
//! peripheral (moved here out of `Peripherals`).

use core::cell::RefCell;

use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_time::{Duration, Timer};
use esp_hal::peripherals::RTC_TIMER;
use esp_hal::rtc_cntl::{Rtc, Rwdt, RwdtStage};

use crate::liveness::LIVENESS;

static RWDT: CriticalSectionMutex<RefCell<Option<Rwdt>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// How often to check that every command-plane task has made progress.
/// Generous relative to each task's own loop cadence (supervisor ~20 ms, ui
/// ~30 ms, director effectively continuous, power/storage 1 Hz) so a
/// healthy 1 Hz task is never mistaken for a hang. Must stay comfortably
/// longer than the worst blocking flash write (which stalls every core-0
/// task, wdt_task included, for its duration) or a healthy persist is
/// misdiagnosed as a hang.
const LIVENESS_CHECK: Duration = Duration::from_millis(8000);

#[embassy_executor::task]
pub async fn wdt_task(rtc_timer: RTC_TIMER<'static>) {
    {
        // Configure once; keep the ZST `Rwdt` handle in the slot for feeding.
        let mut rtc = Rtc::new(rtc_timer);
        rtc.rwdt
            .set_timeout(RwdtStage::Stage0, esp_hal::time::Duration::from_millis(5000));
        rtc.rwdt.enable();
        rtc.rwdt.feed();
        RWDT.lock(|slot| *slot.borrow_mut() = Some(rtc.rwdt));
    }
    log::info!("wdt: RTC watchdog enabled (5 s timeout, 100 ms feed, 8 s liveness check)");

    let mut last_snapshot = LIVENESS.snapshot();
    let mut last_check = embassy_time::Instant::now();

    #[cfg(feature = "debug-prints")]
    static FEED_TICK: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

    loop {
        #[cfg(feature = "debug-prints")]
        {
            let n = FEED_TICK.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            if n % 10 == 0 {
                log::info!("dbg: wdt feed alive");
            }
        }
        if crate::rt::safe_active() {
            // Latched: stop feeding and exit. The RTC watchdog resets the chip
            // shortly; the deadman's fault marker explains why at next boot.
            log::info!("wdt: safe state latched, stopping feed (reboot imminent)");
            break;
        }

        if embassy_time::Instant::now() - last_check >= LIVENESS_CHECK {
            let snapshot = LIVENESS.snapshot();
            if snapshot == last_snapshot {
                log::error!(
                    "wdt: command-plane task hang detected (no liveness progress in {LIVENESS_CHECK:?}) — stopping feed"
                );
                break;
            }
            last_snapshot = snapshot;
            last_check = embassy_time::Instant::now();
        }

        RWDT.lock(|slot| {
            if let Some(wdt) = slot.borrow_mut().as_mut() {
                wdt.feed();
            }
        });
        Timer::after(Duration::from_millis(100)).await;
    }
}
