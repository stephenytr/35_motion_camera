//! Motion director — the only task on core 1 (ARCHITECTURE §6).
//!
//! Owns the TMC2240/5160 over SPI2 (init, currents, 250 ms status polls),
//! builds the RMT step table from `logic::profile` at job arm, and
//! translates commands into jobs (logic::frame_fsm params).
//!
//! TODO(M3+): Rewind-to-zero (needs logic::counters + index_watch home
//! wiring, no home sensor on the bench yet), brownout Recover job.

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
use logic::ramp::BoostRamp;

const STATUS_POLL: HalDuration = HalDuration::from_millis(250);
const RAMP_TICK: HalDuration = HalDuration::from_millis(100);
const IDLE_YIELD: Duration = Duration::from_millis(5);
const DEFAULT_FPS: u8 = 24;
const DEFAULT_EXPOSURE_MS: u32 = 12;

/// Last-armed job settings, kept so Inch/Rewind/Boost can reuse them without
/// the caller having to repeat fps/exposure/shutter every time.
struct JobState {
    fps: u8,
    exposure_ms: u32,
    shutter: bool,
}

/// Build the RMT step table for `params` and arm it; logs and skips on RMT
/// busy (ARCHITECTURE §7.2: caught one frame late by design).
fn arm(params: logic::frame_fsm::FrameParams, frames: Option<u32>, what: &str) {
    let table = logic::profile::build_trapezoid(
        FRAME_USTEPS as usize,
        params.pull_us,
        logic::consts::PULL_ACCEL_FRAC,
    );
    if rmt_step::build_table(&table) {
        rt::arm_job(rt::Job { params, frames });
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

    // M2 demo job: 24 fps, shutter on, infinite — now with the stepper
    // running. The heartbeat applies it at the first FrameStart.
    let mut job = JobState {
        fps: DEFAULT_FPS,
        exposure_ms: DEFAULT_EXPOSURE_MS,
        shutter: true,
    };
    let params = logic::frame_fsm::params_for(job.fps as f32, job.exposure_ms, job.shutter);
    arm(params, None, "boot default job");
    info!("director: armed default job 24 fps, shutter on, stepper on, infinite");

    // Boost ramp base tracks `job.fps`; live-updated every RAMP_TICK while
    // boosting (or coasting back down) so the pull speed slews smoothly
    // instead of snapping (SPECS §4.2, logic::ramp).
    let mut boost = BoostRamp::new(job.fps as f32, 24.0, 48.0);

    let mut last_status = Instant::now();
    let mut last_ramp = Instant::now();

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
                let params = logic::frame_fsm::params_for(fps as f32, job.exposure_ms, shutter);
                arm(params, frames, "Run");
            }
            Some(Command::Stop) => {
                info!("director: Stop — parking at next frame boundary");
                heartbeat::request_stop();
            }
            Some(Command::Inch { frames }) => {
                info!("director: Inch {frames} frame(s)");
                tmc.set_dir(true);
                let params = logic::frame_fsm::params_for(job.fps as f32, job.exposure_ms, job.shutter);
                arm(params, Some(frames), "Inch");
            }
            Some(Command::Rewind { to_zero }) => {
                if to_zero {
                    // Needs logic::counters + index_watch home wiring — no
                    // home sensor on the bench yet. Refuse rather than spin
                    // indefinitely with no stop condition.
                    warn!("director: Rewind to_zero — not implemented (no home sensor), ignoring");
                } else {
                    info!("director: Rewind — running reverse until Stop");
                    tmc.set_dir(false);
                    let params = logic::frame_fsm::params_for(job.fps as f32, job.exposure_ms, job.shutter);
                    arm(params, None, "Rewind");
                }
            }
            Some(Command::SetFps(fps)) => {
                info!("director: SetFps {fps}");
                job.fps = fps;
                boost.set_base(fps as f32);
            }
            Some(Command::SetExposure(ms)) => {
                info!("director: SetExposure {ms} ms");
                job.exposure_ms = ms;
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
                if last_status.elapsed() >= STATUS_POLL {
                    last_status = Instant::now();
                    if let Some(code) = tmc.poll_status() {
                        warn!(
                            "director: TMC fault, full DRV_STATUS={:#010x} (code={code:#06x})",
                            tmc.read_drv_status()
                        );
                        rt::safe_state();
                        let _ = events.try_send(Event::Fault(ErrorCode::Driver(code)));
                    }
                }
                if last_ramp.elapsed() >= RAMP_TICK && !boost.at_target() {
                    let dt_s = last_ramp.elapsed().as_micros() as f32 / 1_000_000.0;
                    last_ramp = Instant::now();
                    let fps = boost.step(dt_s);
                    let params =
                        logic::frame_fsm::params_for(fps, job.exposure_ms, job.shutter);
                    heartbeat::update_live_params(params);
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
