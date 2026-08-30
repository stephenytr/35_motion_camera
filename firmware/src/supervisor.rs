//! Supervisor task (core 0): applies the interlock matrix (logic::interlock),
//! latches faults, feeds the RTC watchdog, translates UI commands into
//! director queue entries (ARCHITECTURE §5.1–5.3).
//!
//! Counter wiring (SPECS §9.4, §12): exposure counting is *poll-based* here —
//! the heartbeat ISR increments `status::STATUS.exposed_count` directly (the
//! event channel is per-core-CS and drops cross-core sends at frame rates;
//! decision log #23), and this task polls it every 20 ms along with the RT
//! film position (rewind frames decrement the counter). Roll-end auto-stops
//! via the interlock matrix; counters are persisted at job boundaries only
//! (idle-gated writes).

use embassy_time::{Duration, with_timeout};
use log::{info, warn};

use crate::command::{Command, CmdProducer};
use crate::fault::{Event, EventChannel};
use crate::settings_store;
use crate::status::{State, Status};
use logic::settings::Track;

/// Counter-poll cadence (roll-end must catch within a frame at any fps).
const COUNTER_POLL: Duration = Duration::from_millis(20);

#[embassy_executor::task]
pub async fn supervisor_task(
    events: &'static EventChannel,
    mut cmds: CmdProducer,
    status: &'static Status,
) {
    status.set_state(State::Idle);
    // SPECS §11 power-on self-test, bench scope: report the door interlock
    // state (the index/battery/self-test legs land with their hardware).
    info!(
        "supervisor: up, state = {:?}, door = {}, position = {} frames",
        status.state(),
        if crate::rt::door::is_open() { "OPEN" } else { "closed" },
        crate::rt::position::frames(),
    );

    // Cumulative exposed count of the active track (SPECS §9.4). Local to
    // this task; persisted to the settings store at job boundaries.
    let mut exposed: u32 = 0;
    // Last-seen values for delta detection.
    let mut last_pos: u32 = crate::rt::position::frames();
    let mut last_count: u32 = status.exposed_count.load(core::sync::atomic::Ordering::Relaxed);

    /// Which persisted counter slot the active track maps to (0=A, 1=B).
    fn track_idx() -> u8 {
        let s = settings_store::settings();
        match (s.mode, s.track) {
            (logic::settings::TrackMode::Dual, Track::B) => 1,
            _ => 0,
        }
    }

    loop {
        match with_timeout(COUNTER_POLL, events.receive()).await {
            Ok(Event::JobComplete) => {
                settings_store::set_exposed(track_idx(), exposed);
                status.set_state(State::Idle);
                info!("supervisor: JobComplete -> IDLE (exposed {exposed})");
            }
            Ok(Event::CounterZero) => {
                // Rewind-to-zero reached the datum (SPECS §10 step 4).
                exposed = 0;
                settings_store::set_exposed(track_idx(), 0);
                status.set_state(State::Idle);
                info!("supervisor: CounterZero — film at datum");
            }
            Ok(Event::DoorOpen) => {
                // SPECS §11: door events persist counters + settings.
                settings_store::set_exposed(track_idx(), exposed);
                status.door_open.store(true, core::sync::atomic::Ordering::Relaxed);
                status.set_state(State::Door);
                info!("supervisor: DoorOpen -> DOOR");
            }
            Ok(Event::DoorClosed) => {
                status.door_open.store(false, core::sync::atomic::Ordering::Relaxed);
                info!("supervisor: DoorClosed");
            }
            Ok(Event::Fault(code)) => {
                status.set_state(State::Error);
                status.fault.store(code.code(), core::sync::atomic::Ordering::Relaxed);
                info!("supervisor: Fault({code:?}) -> ERROR");
            }
            Ok(Event::IndexTick) => {
                // Sprocket index edge accepted (ARCHITECTURE §4.3) — up to
                // ~3.6 Hz at 24 fps, so no log line; status-only.
            }
            Ok(Event::SettingsChanged) => info!("supervisor: SettingsChanged"),
            Err(_) => {
                // 20 ms counter poll (see module docs).
                let count = status
                    .exposed_count
                    .load(core::sync::atomic::Ordering::Relaxed);
                if count != last_count {
                    exposed = exposed.saturating_add(count - last_count);
                    last_count = count;
                    // Roll end (SPECS §11): finish the current frame, park.
                    if exposed >= settings_store::settings().roll_frames {
                        let r = logic::interlock::evaluate(logic::interlock::Condition::FilmEnd);
                        warn!("supervisor: ROLL END at frame {exposed} — parking");
                        if let Some(code) = r.latch {
                            status
                                .fault
                                .store(code.code(), core::sync::atomic::Ordering::Relaxed);
                        }
                        let _ = cmds.enqueue(Command::Stop);
                    }
                }
                // Rewind decrements the counter with the film (SPECS §10
                // step 4: "counter decrements to zero").
                let pos = crate::rt::position::frames();
                if pos < last_pos {
                    exposed = exposed.saturating_sub(last_pos - pos);
                }
                last_pos = pos;
            }
        }
    }
}
