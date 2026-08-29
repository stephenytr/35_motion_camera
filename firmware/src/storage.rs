//! Settings/counter persistence (ARCHITECTURE §12): idle-gated flash writes only
//! (run stop, door events, roll end, settings change), ping-pong 4 KB sectors,
//! versioned `logic::settings::Settings` records + CRC32.
//!
//! M1 scope: 1 Hz idle-gate checker proving invariant 5's gating rule — the
//! task only *pretends* to persist when the transport is idle.

use embassy_time::{Duration, Ticker};
use log::info;

use crate::status::{State, Status};

#[embassy_executor::task]
pub async fn storage_task(status: &'static Status) {
    info!("storage: up, idle-gated persistence checker at 1 Hz");
    let mut ticker = Ticker::every(Duration::from_secs(1));

    loop {
        ticker.next().await;
        match status.state() {
            State::Idle | State::Door | State::Error => {
                // ARCHITECTURE §12: flash writes only at idle boundaries.
                info!("storage: state allows writes, nothing dirty (stub)");
            }
            _ => {
                info!("storage: transport active, writes gated");
            }
        }
    }
}
