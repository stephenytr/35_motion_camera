//! UI task (core 0): buttons, menu model (logic::menu), OLED redraw ≤ 10 Hz,
//! main-screen layout per SPECS §9.4.
//!
//! M1 scope: 10 Hz ticker proving the task cadence; logs once per second with
//! the live `Status` (what the OLED will show in M4).

use embassy_time::{Duration, Ticker};
use log::info;

use crate::status::{State, Status};

#[embassy_executor::task]
pub async fn ui_task(status: &'static Status) {
    info!("ui: up, redraw ticker at 10 Hz");
    let mut ticker = Ticker::every(Duration::from_millis(100));
    let mut tick = 0u32;

    loop {
        ticker.next().await;
        tick = tick.wrapping_add(1);

        if tick % 10 == 0 {
            let state = status.state();
            let frames = status.frames_exposed.load(core::sync::atomic::Ordering::Relaxed);
            let vb = status.vbat_mv.load(core::sync::atomic::Ordering::Relaxed);
            info!("ui: redraw state={state:?} frames={frames} vbat={vb}mV");
        }
    }
}

/// What the main screen shows per state (SPECS §9.4); filled in during M4.
#[allow(dead_code)]
pub fn main_screen(_state: State) -> &'static str {
    "35mm 1.5P"
}
