//! Power task (core 0): ADC supervision (ARCHITECTURE §5.1).
//! VBAT / IPROPI thresholds 19.8 V / 18.3 V (SPECS), faulting into the
//! interlock matrix.
//!
//! M1 scope: 1 Hz cadence; ADC driver lands in M3, so it publishes a stub
//! voltage into `Status` for the UI to display.

use embassy_time::{Duration, Ticker};
use log::{info, warn};

use crate::status::Status;

#[embassy_executor::task]
pub async fn power_task(status: &'static Status) {
    info!("power: up, 1 Hz ADC supervision");
    let mut ticker = Ticker::every(Duration::from_secs(1));

    loop {
        ticker.next().await;

        // TODO(M3): read VBAT via ADC; stub 24.0 V until the driver exists.
        let vb_mv = 24_000u32;
        status.vbat_mv.store(vb_mv, core::sync::atomic::Ordering::Relaxed);

        // SPECS §11 / §6.2: two independent thresholds. 19.8 V only warns
        // (display, no stop); 18.3 V is the auto-stop condition — wired
        // through `logic::interlock::evaluate` in the supervisor's
        // `Event::Fault` handler, which now actually enqueues `Stop`
        // (audit finding: previously the fault only latched a code and
        // the transport kept running).
        let warn_now = vb_mv < 19_800;
        status
            .batt_warn
            .store(warn_now, core::sync::atomic::Ordering::Relaxed);
        if warn_now {
            warn!("power: vbat={vb_mv}mV — LOW BATTERY warn");
        }
        if vb_mv < 18_300 {
            crate::fault::raise(logic::interlock::ErrorCode::CriticalBattery);
        }

        crate::liveness::LIVENESS.bump_power();
    }
}
