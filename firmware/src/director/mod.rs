//! Motion director — the only task on core 1 (ARCHITECTURE §6).
//!
//! Owns the TMC2240/5160 over SPI2 (init, currents, 250 ms status polls),
//! builds the RMT step table from `logic::profile` at job arm, and
//! translates commands into jobs (logic::frame_fsm params).
//!
//! Script jobs (SPECS §10) decompose into job chains issued on
//! `heartbeat::job_done_take()`: leader marks → leader gap → park, and
//! track-B setup. A `logic::transport::Transport` model shadows the RT
//! position accumulator and keeps the pass-end datum for track-B
//! re-alignment.
//!
//! `Command::Recover` (brownout: creep to next index edge and park) is
//! auto-issued by the supervisor on a brownout reset reason (main.rs) as
//! well as reachable manually; bench-exercised with synthetic index edges
//! until the sensor lands. TODO(M3+): index-present boot self-test.

use embassy_time::{Duration, Timer};
use esp_hal::time::{Duration as HalDuration, Instant};
use log::{info, warn};

use crate::command::{CmdConsumer, Command};
use crate::drivers::{rmt_step, tmc::Tmc};
use crate::fault::{ErrorCode, Event, EventChannel};
use crate::rt;
use crate::rt::heartbeat;
use crate::status::Status;
use logic::consts::FRAME_USTEPS;
use logic::position::Direction;
use logic::ramp::BoostRamp;
use logic::transport::{ScriptPhase, Transport, LEADER_GAP_FRAMES, LEADER_MARK_FRAMES};

const STATUS_POLL: HalDuration = HalDuration::from_millis(250);
const RAMP_TICK: HalDuration = HalDuration::from_millis(100);
const IDLE_YIELD: Duration = Duration::from_millis(5);
const DEFAULT_FPS: u8 = 24;
const DEFAULT_EXPOSURE_MS: u32 = 12;
/// Creep speed for blind advances (leader gap, track-B setup) — SPECS §10
/// does not pin a number; 6 fps keeps the cadence visibly slow on the bench.
const CREEP_FPS: f32 = 6.0;

/// Last-armed job settings, kept so Inch/Rewind/Boost can reuse them without
/// the caller having to repeat fps/exposure/shutter every time.
struct JobState {
    fps: u8,
    exposure_ms: u32,
    shutter: bool,
}

/// Build the RMT step table for `params` and arm it; logs and skips on RMT
/// busy (ARCHITECTURE §7.2: caught one frame late by design). Direction is
/// applied to the TMC DIR pin *and* the RT position accumulator before the
/// heartbeat can apply the job.
fn arm(params: logic::frame_fsm::FrameParams, frames: Option<u32>, dir: Direction, what: &str) {
    let table = logic::profile::build_trapezoid(
        FRAME_USTEPS as usize,
        params.pull_us,
        logic::consts::PULL_ACCEL_FRAC,
    );
    if rmt_step::build_table(&table) {
        rt::arm_job(rt::Job {
            params,
            frames,
            direction: dir,
        });
    } else {
        warn!("director: RMT busy — skipping {what} arm");
    }
}

#[embassy_executor::task]
pub async fn director_task(
    events: &'static EventChannel,
    mut cmds: CmdConsumer,
    _status: &'static Status,
    mut tmc: Tmc,
) {
    info!(
        "director: up on core {}",
        esp_hal::system::Cpu::current() as usize
    );

    // TMC bring-up + comms self-test before any job is armed.
    let ioin = tmc.init();
    tmc.clear_gstat();
    tmc.set_dir(true);
    info!("tmc: self-test IOIN version {:#04x}", (ioin >> 24) & 0xFF);

    // SPECS §11 power-on: self-test, then IDLE — motion starts on RUN
    // only. (The M2/M3 bring-up auto-armed a demo job here; with the UI
    // landed, boot must sit still until commanded.)
    let mut job = JobState {
        fps: DEFAULT_FPS,
        exposure_ms: DEFAULT_EXPOSURE_MS,
        shutter: true,
    };
    info!("director: idle at boot — press RUN to start");

    // Boost ramp base tracks `job.fps`; live-updated every RAMP_TICK while
    // boosting (or coasting back down) so the pull speed slews smoothly
    // instead of snapping (SPECS §4.2, logic::ramp).
    let mut boost = BoostRamp::new(job.fps as f32, 24.0, 48.0);

    // Transport procedure shadow (SPECS §10).
    let mut transport = Transport::new();

    let mut last_status = Instant::now();
    let mut last_ramp = Instant::now();
    let mut last_advisory: u32 = 0;

    loop {
        match cmds.dequeue() {
            Some(Command::SelfTest) => {
                let ioin = tmc.read_ioin();
                let st = tmc.read_drv_status();
                info!("director: SelfTest IOIN={ioin:#010x} DRV_STATUS={st:#010x}");
            }
            Some(Command::Run { fps, frames, shutter }) => {
                info!("director: Run fps={fps} frames={frames:?} shutter={shutter}");
                job = JobState { fps, exposure_ms: job.exposure_ms, shutter };
                boost.set_base(fps as f32);
                tmc.set_dir(true); // Run always advances film forward
                rt::position::set_direction(Direction::Forward);
                let params = logic::frame_fsm::params_for(fps as f32, job.exposure_ms, shutter);
                arm(params, frames, Direction::Forward, "Run");
            }
            Some(Command::Stop) => {
                info!("director: Stop — parking at next frame boundary");
                heartbeat::request_stop();
            }
            Some(Command::Inch { frames }) => {
                info!("director: Inch {frames} frame(s)");
                tmc.set_dir(true);
                rt::position::set_direction(Direction::Forward);
                let params = logic::frame_fsm::params_for(job.fps as f32, job.exposure_ms, job.shutter);
                arm(params, Some(frames), Direction::Forward, "Inch");
            }
            Some(Command::Rewind { to_zero }) => {
                transport.sync_position(rt::position::usteps());
                let remaining = transport.rewind_plan();
                if to_zero && remaining == 0 {
                    info!("director: Rewind to zero — already at datum, nothing to do");
                } else {
                    let frames = if to_zero { Some(remaining) } else { None };
                    info!("director: Rewind to_zero={to_zero} frames={frames:?}");
                    tmc.set_dir(false);
                    rt::position::set_direction(Direction::Reverse);
                    arm(
                        logic::frame_fsm::rewind_params(),
                        frames,
                        Direction::Reverse,
                        "rewind",
                    );
                }
            }
            Some(Command::LeaderMark) => {
                info!("director: leader marks — {LEADER_MARK_FRAMES} frames @ {CREEP_FPS:.0} fps, shutter");
                transport.set_phase(ScriptPhase::LeaderMarks);
                tmc.set_dir(true);
                rt::position::set_direction(Direction::Forward);
                let params =
                    logic::frame_fsm::params_for(CREEP_FPS, job.exposure_ms, true);
                arm(params, Some(LEADER_MARK_FRAMES), Direction::Forward, "leader marks");
            }
            Some(Command::TrackBSetup) => {
                transport.sync_position(rt::position::usteps());
                let frames = transport.track_b_plan();
                if frames == 0 {
                    info!("director: TrackBSetup — no pass recorded, nothing to advance");
                } else {
                    info!("director: TrackBSetup — advancing {frames} frames to pass end");
                    transport.set_phase(ScriptPhase::TrackBAdvance);
                    tmc.set_dir(true);
                    rt::position::set_direction(Direction::Forward);
                    let params =
                        logic::frame_fsm::params_for(CREEP_FPS, job.exposure_ms, false);
                    arm(params, Some(frames), Direction::Forward, "track B setup");
                }
            }
            Some(Command::Recover) => {
                // Brownout recovery (SPECS §11): creep forward and park at
                // the next index edge — the edge acceptance path requests
                // the park via rt::index. `frames: None`: the edge is the
                // stop condition. Needs the sensor; bench exercises it
                // with synthetic edges (IndexEdgeAt).
                info!("director: Recover — creeping to next index edge");
                transport.sync_position(rt::position::usteps());
                tmc.set_dir(true);
                rt::position::set_direction(Direction::Forward);
                rt::index::arm_recover_stop(true);
                let params =
                    logic::frame_fsm::params_for(CREEP_FPS, job.exposure_ms, false);
                arm(params, None, Direction::Forward, "recover creep");
            }
            Some(Command::IndexArm) => {
                // TEMP bench hook (see command.rs): runs on core 1 where
                // the index state's CS mutex is safe.
                info!("director: index watchdog armed (bench)");
                rt::index::set_enabled(true);
            }
            Some(Command::IndexEdgeAt(usteps)) => {
                // TEMP bench hook: synthetic index edge at `usteps`.
                rt::index::debug_edge_at(usteps as i32);
            }
            Some(Command::SetFps(fps)) => {
                info!("director: SetFps {fps}");
                job.fps = fps;
                boost.set_base(fps as f32);
                // Persist at the next idle boundary (ARCHITECTURE §12).
                let mut s = crate::settings_store::settings();
                s.fps = fps as f32;
                crate::settings_store::set_settings(s);
            }
            Some(Command::SetExposure(ms)) => {
                info!("director: SetExposure {ms} ms");
                job.exposure_ms = ms;
                let mut s = crate::settings_store::settings();
                s.exposure_ms = ms;
                crate::settings_store::set_settings(s);
            }
            Some(Command::Boost(on)) => {
                info!("director: Boost {on}");
                if on {
                    boost.boost_on();
                } else {
                    boost.boost_off();
                }
            }
            None => {
                // Script chaining: a parked job means the next chain step.
                if heartbeat::job_done_take() {
                    let phase = transport.phase();
                    transport.sync_position(rt::position::usteps());
                    match phase {
                        ScriptPhase::LeaderMarks => {
                            info!("director: leader marks done — advancing {LEADER_GAP_FRAMES}-frame gap");
                            transport.set_phase(ScriptPhase::LeaderGap);
                            tmc.set_dir(true);
                            rt::position::set_direction(Direction::Forward);
                            let params =
                                logic::frame_fsm::params_for(CREEP_FPS, job.exposure_ms, false);
                            arm(params, Some(LEADER_GAP_FRAMES), Direction::Forward, "leader gap");
                        }
                        ScriptPhase::LeaderGap => {
                            info!("director: leader procedure complete (marks + gap)");
                        }
                        ScriptPhase::TrackBAdvance => {
                            info!("director: track B setup complete");
                        }
                        ScriptPhase::Idle => {}
                    }
                    transport.on_job_end(rt::position::direction());
                }

                if last_status.elapsed() >= STATUS_POLL {
                    last_status = Instant::now();
                    let st = tmc.read_drv_status();
                    if st & crate::drivers::tmc::HARD_FAULT_MASK != 0 {
                        warn!(
                            "director: TMC hard fault, DRV_STATUS={st:#010x} (code={:#06x})",
                            (st >> 16) as u16
                        );
                        // Full RT-plane stop, matching the door/JAM paths:
                        // driving the actuators off alone leaves the
                        // heartbeat cycling the shutter and counting
                        // exposures while the supervisor sits in ERROR.
                        rt::safe_state();
                        rt::heartbeat::halt();
                        rt::shutter::disarm();
                        rt::deadman::disarm();
                        let _ = events.try_send(Event::Fault(ErrorCode::Driver((st >> 16) as u16)));
                    } else if st & crate::drivers::tmc::ADVISORY_MASK != 0
                        && st & crate::drivers::tmc::ADVISORY_MASK != last_advisory
                    {
                        // S2G/OL false-positive territory (tmc.rs): log on
                        // change only, never latch.
                        last_advisory = st & crate::drivers::tmc::ADVISORY_MASK;
                        warn!(
                            "director: TMC advisory S2G/OL bits {:#010x} (log-only)",
                            last_advisory
                        );
                    }
                }
                if last_ramp.elapsed() >= RAMP_TICK && !heartbeat::is_parked() && !boost.at_target() {
                    let dt_s = last_ramp.elapsed().as_micros() as f32 / 1_000_000.0;
                    last_ramp = Instant::now();
                    let fps = boost.step(dt_s);
                    let params =
                        logic::frame_fsm::params_for(fps, job.exposure_ms, job.shutter);
                    // The step table must track the slewing cadence, or the
                    // transfer overruns the frame at faster fps (RmtBusy).
                    // build_table's poll-and-reclaim makes this safe mid-job.
                    let table = logic::profile::build_trapezoid(
                        FRAME_USTEPS as usize,
                        params.pull_us,
                        logic::consts::PULL_ACCEL_FRAC,
                    );
                    if rmt_step::build_table(&table) {
                        heartbeat::update_live_params(params);
                    } else {
                        // Transfer in flight — retry on the next tick; the
                        // ramp is far slower than the tick rate.
                        warn!("director: ramp tick deferred (RMT transfer in flight)");
                    }
                } else if last_ramp.elapsed() >= RAMP_TICK {
                    last_ramp = Instant::now();
                }
                // Cooperative poll: the director has this executor to
                // itself, but still yields so interrupts/timers stay happy.
                Timer::after(IDLE_YIELD).await;
            }
        }
    }
}
