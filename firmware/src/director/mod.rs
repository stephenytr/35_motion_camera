//! Motion director — the only task on core 1 (ARCHITECTURE §6).
//!
//! Owns the TMC2240/5160 over SPI2 (init, currents, 250 ms status polls),
//! builds the RMT step table from `logic::profile` at job arm, and
//! translates commands into jobs (logic::frame_fsm params).
//!
//! TODO(M3+): Stop = park-at-frame-end, boost ramp (logic::ramp), inch/
//! rewind creep jobs, brownout Recover job, index watchdog wiring.

use embassy_time::{Duration, Timer};
use esp_hal::time::{Duration as HalDuration, Instant};
use log::{info, warn};

use crate::command::{CmdConsumer, Command};
use crate::drivers::{rmt_step, tmc::Tmc};
use crate::fault::{ErrorCode, Event, EventChannel};
use crate::rt;
use crate::status::Status;
use logic::consts::FRAME_USTEPS;

const STATUS_POLL: HalDuration = HalDuration::from_millis(250);
const IDLE_YIELD: Duration = Duration::from_millis(5);

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
    let params = logic::frame_fsm::params_for(24.0, 12, true);
    let table = logic::profile::build_trapezoid(
        FRAME_USTEPS as usize,
        params.pull_us,
        logic::consts::PULL_ACCEL_FRAC,
    );
    if !rmt_step::build_table(&table) {
        warn!("director: RMT busy at boot — table not loaded");
    }
    rt::arm_job(rt::Job {
        params,
        frames: None,
    });
    info!("director: armed default job 24 fps, shutter on, stepper on, infinite");

    let mut last_status = Instant::now();

    loop {
        match cmds.dequeue() {
            Some(Command::SelfTest) => {
                let ioin = tmc.read_ioin();
                let st = tmc.read_drv_status();
                info!("director: SelfTest IOIN={ioin:#010x} DRV_STATUS={st:#010x}");
            }
            Some(Command::Run { fps, frames, shutter }) => {
                info!("director: Run fps={fps} frames={frames:?} shutter={shutter}");
                let params = logic::frame_fsm::params_for(fps as f32, 12, shutter);
                let table = logic::profile::build_trapezoid(
                    FRAME_USTEPS as usize,
                    params.pull_us,
                    logic::consts::PULL_ACCEL_FRAC,
                );
                if rmt_step::build_table(&table) {
                    rt::arm_job(rt::Job {
                        params,
                        frames,
                    });
                } else {
                    warn!("director: RMT busy — skipping Run arm");
                }
            }
            Some(Command::Stop) => {
                info!("director: Stop — park-at-frame-end arrives in M3b");
            }
            Some(Command::Inch { frames }) => {
                info!("director: Inch {frames} frame(s) — M3 (creep job)");
            }
            Some(Command::Rewind { to_zero }) => {
                info!("director: Rewind to_zero={to_zero} — M3 (rewind job)");
            }
            Some(Command::SetFps(fps)) => info!("director: SetFps {fps}"),
            Some(Command::SetExposure(ms)) => info!("director: SetExposure {ms} ms"),
            Some(Command::Boost(on)) => info!("director: Boost {on}"),
            None => {
                if last_status.elapsed() >= STATUS_POLL {
                    last_status = Instant::now();
                    if let Some(code) = tmc.poll_status() {
                        warn!("director: TMC fault DRV_STATUS={code:#06x}");
                        rt::safe_state();
                        let _ = events.try_send(Event::Fault(ErrorCode::Driver(code)));
                    }
                }
                // Cooperative poll: the director has this executor to
                // itself, but still yields so interrupts/timers stay happy.
                Timer::after(IDLE_YIELD).await;
            }
        }
    }
}
