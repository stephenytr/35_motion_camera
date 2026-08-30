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
use crate::ui::{TransportAction, UiEvent, UI_EVENTS};
use logic::menu::MenuItem;
use logic::settings::Track;

/// Counter-poll cadence (roll-end must catch within a frame at any fps).
const COUNTER_POLL: Duration = Duration::from_millis(20);

#[embassy_executor::task]
pub async fn supervisor_task(
    events: &'static EventChannel,
    mut cmds: CmdProducer,
    status: &'static Status,
    brownout: bool,
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

    // Brownout auto-recover (SPECS §11): `main` flags a VDD-dip reset; if
    // the door is shut, creep to the next index edge and park — same job a
    // manual Recover command runs. JobComplete below returns us to IDLE.
    if brownout {
        if crate::rt::door::is_open() {
            warn!("supervisor: brownout recovery skipped — door open");
        } else {
            warn!("supervisor: brownout detected — auto-recovering (creep to next index edge)");
            let _ = cmds.enqueue(Command::Recover);
        }
    }

    // Cumulative exposed count of the active track (SPECS §9.4). Local to
    // this task; persisted to the settings store at job boundaries.
    let mut exposed: u32 = 0;
    let mut boosting: bool = false;
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

    /// UI command translation (ARCHITECTURE §8): validate against the
    /// interlock matrix, then enqueue the director command.
    fn handle_ui(
        ev: UiEvent,
        cmds: &mut CmdProducer,
        status: &'static Status,
        boosting: &mut bool,
    ) {
        match ev {
            UiEvent::RunToggle => {
                if crate::rt::door::is_open() {
                    warn!("supervisor: RUN ignored — door open (interlock)");
                    return;
                }
                if crate::rt::heartbeat::is_parked() {
                    let s = settings_store::settings();
                    let fps = (s.fps + 0.5) as u8;
                    info!("supervisor: RUN -> Run {fps} fps, infinite");
                    let _ = cmds.enqueue(Command::Run { fps, frames: None, shutter: true });
                } else {
                    info!("supervisor: RUN -> Stop");
                    let _ = cmds.enqueue(Command::Stop);
                }
            }
            UiEvent::Adjust { item, up } => {
                let mut s = settings_store::settings();
                match item {
                    MenuItem::Fps => {
                        let step = if up { logic::consts::FPS_STEP } else { -logic::consts::FPS_STEP };
                        // Round before stepping: heals legacy 0.5-step
                        // values persisted by older builds (director SetFps
                        // already writes integers back, but the display and
                        // the motor must never disagree about the step).
                        s.fps = ((s.fps + 0.5) as u8 as f32 + step)
                            .clamp(logic::consts::FPS_MIN, logic::consts::FPS_MAX);
                        settings_store::set_settings(s);
                        info!("supervisor: fps = {:.0}", s.fps);
                        let _ = cmds.enqueue(Command::SetFps(s.fps as u8));
                    }
                    MenuItem::Exposure => {
                        let next = s.exposure_ms as i64 + if up { 1 } else { -1 };
                        s.exposure_ms = next
                            .clamp(
                                logic::consts::EXPOSURE_MIN_MS as i64,
                                logic::consts::EXPOSURE_MAX_MS as i64,
                            )
                            as u32;
                        settings_store::set_settings(s);
                        info!("supervisor: exposure = {} ms", s.exposure_ms);
                        let _ = cmds.enqueue(Command::SetExposure(s.exposure_ms));
                    }
                    MenuItem::Roll => {
                        s.roll_frames = if up {
                            s.roll_frames.saturating_add(10)
                        } else {
                            s.roll_frames.saturating_sub(10).max(1)
                        };
                        settings_store::set_settings(s);
                        info!("supervisor: roll = {} frames", s.roll_frames);
                    }
                    MenuItem::Track => {
                        s.track = match s.track {
                            Track::A => Track::B,
                            Track::B => Track::A,
                        };
                        settings_store::set_settings(s);
                        info!("supervisor: track = {:?}", s.track);
                    }
                    MenuItem::Boost => {
                        // Direction-consistent, like every other adjust:
                        // ▲ = on, ▼ = off (previously toggled on *either*
                        // press, so ▼ from OFF could turn boost on).
                        *boosting = up;
                        status
                            .boost
                            .store(*boosting, core::sync::atomic::Ordering::Relaxed);
                        info!("supervisor: boost = {}", if *boosting { "on" } else { "off" });
                        let _ = cmds.enqueue(Command::Boost(*boosting));
                    }
                    MenuItem::Settings => {
                        let step: i64 = if up { 5 } else { -5 };
                        s.hold_pct = (s.hold_pct as i64 + step).clamp(0, 100) as u32;
                        settings_store::set_settings(s);
                        info!("supervisor: hold = {}%", s.hold_pct);
                    }
                    _ => {}
                }
            }
            UiEvent::Transport(action) => {
                if crate::rt::door::is_open() {
                    warn!("supervisor: transport ignored — door open (interlock)");
                    return;
                }
                match action {
                    TransportAction::Leader => {
                        info!("supervisor: transport -> track leader");
                        let _ = cmds.enqueue(Command::LeaderMark);
                    }
                    TransportAction::Rewind => {
                        info!("supervisor: transport -> rewind to zero");
                        let _ = cmds.enqueue(Command::Rewind { to_zero: true });
                    }
                    TransportAction::TrackBSetup => {
                        info!("supervisor: transport -> track B setup");
                        let _ = cmds.enqueue(Command::TrackBSetup);
                    }
                }
            }
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
                // UI events first (same-core, cheap).
                while let Ok(ev) = UI_EVENTS.try_receive() {
                    handle_ui(ev, &mut cmds, status, &mut boosting);
                }

                let count = status
                    .exposed_count
                    .load(core::sync::atomic::Ordering::Relaxed);
                if count != last_count {
                    exposed = exposed.saturating_add(count - last_count);
                    last_count = count;
                    status
                        .counter_exposed
                        .store(exposed, core::sync::atomic::Ordering::Relaxed);
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
                    status
                        .counter_exposed
                        .store(exposed, core::sync::atomic::Ordering::Relaxed);
                }
                last_pos = pos;
            }
        }
    }
}
