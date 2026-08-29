//! Power task (core 0): ADC supervision (ARCHITECTURE §5.1).
//! VBAT / IPROPI thresholds 19.8 V / 18.3 V (SPECS), faulting into the
//! interlock matrix.
//!
//! M1 scope: 1 Hz cadence; ADC driver lands in M3, so it publishes a stub
//! voltage into `Status` for the UI to display.

use embassy_time::{Duration, Ticker};
use log::info;

use crate::fault::{Event, EventChannel};
use crate::status::Status;

#[embassy_executor::task]
pub async fn power_task(events: &'static EventChannel, status: &'static Status) {
    info!("power: up, 1 Hz ADC supervision");
    let mut ticker = Ticker::every(Duration::from_secs(1));

    loop {
        ticker.next().await;

        // TODO(M3): read VBAT via ADC; stub 24.0 V until the driver exists.
        let vb_mv = 24_000u32;
        status.vbat_mv.store(vb_mv, core::sync::atomic::Ordering::Relaxed);
        info!("power: vbat={vb_mv}mV (stub, ok)");

        if vb_mv < 19_800 {
            let _ = events.try_send(Event::Fault(logic::interlock::ErrorCode::CriticalBattery));
        }
    }
}
