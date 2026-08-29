//! RTC-watchdog feeder (ARCHITECTURE §4.4, §5.1): feeds the RTC watchdog
//! every 100 ms while the system is healthy. When the deadman (or door) ISR
//! latches safe state, this task sees the latch, stops feeding, and lets the
//! watchdog reboot the chip — the deadman's fault marker is then read at boot.
//!
//! The timeout is 2 s: plenty of headroom over the 100 ms feed cadence, short
//! enough that a hung system comes back quickly.
//!
//! Note `esp_hal::init` disables the RWDT at boot; this task re-arms it. The
//! `Rwdt` handle is the ZST field of `Rtc`, which takes the RTC_TIMER
//! peripheral (moved here out of `Peripherals`).

use core::cell::RefCell;

use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_time::{Duration, Timer};
use esp_hal::peripherals::RTC_TIMER;
use esp_hal::rtc_cntl::{Rtc, Rwdt, RwdtStage};

static RWDT: CriticalSectionMutex<RefCell<Option<Rwdt>>> =
    CriticalSectionMutex::new(RefCell::new(None));

#[embassy_executor::task]
pub async fn wdt_task(rtc_timer: RTC_TIMER<'static>) {
    {
        // Configure once; keep the ZST `Rwdt` handle in the slot for feeding.
        let mut rtc = Rtc::new(rtc_timer);
        rtc.rwdt
            .set_timeout(RwdtStage::Stage0, esp_hal::time::Duration::from_millis(2000));
        rtc.rwdt.enable();
        rtc.rwdt.feed();
        RWDT.lock(|slot| *slot.borrow_mut() = Some(rtc.rwdt));
    }
    log::info!("wdt: RTC watchdog enabled (2 s timeout, 100 ms feed)");

    loop {
        if crate::rt::safe_active() {
            // Latched: stop feeding and exit. The RTC watchdog resets the chip
            // shortly; the deadman's fault marker explains why at next boot.
            log::info!("wdt: safe state latched, stopping feed (reboot imminent)");
            break;
        }
        RWDT.lock(|slot| {
            if let Some(wdt) = slot.borrow_mut().as_mut() {
                wdt.feed();
            }
        });
        Timer::after(Duration::from_millis(100)).await;
    }
}
