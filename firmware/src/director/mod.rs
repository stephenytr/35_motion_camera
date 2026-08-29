//! Motion director — the only task on core 1 (ARCHITECTURE §6).
//!
//! M2a scope: arms the RT plane's default job at boot and translates `Run`
//! commands into jobs (logic::frame_fsm params). Drains the rest of the
//! command queue as logged stubs.
//!
//! TODO(M2b+): shutter pair (peak-hold/exposure-end), Stop = park-at-frame-end,
//! own TMC5160 over SPI2 (init, currents, 250 ms status polls), run the boost
//! ramp (logic::ramp), rebuild RMT step tables (logic::profile), brownout
//! Recover job.

use embassy_time::{Duration, Timer};
use log::info;

use crate::command::{CmdConsumer, Command};
use crate::fault::EventChannel;
use crate::rt;
use crate::status::Status;

#[embassy_executor::task]
pub async fn director_task(
    _events: &'static EventChannel,
    mut cmds: CmdConsumer,
    _status: &'static Status,
) {
    info!(
        "director: up on core {}",
        esp_hal::system::Cpu::current() as usize
    );

    // M2 demo job: 24 fps, shutter on, infinite — runs until M2b/M3 stop paths
    // exist. The heartbeat applies it at the first FrameStart.
    rt::arm_job(rt::Job {
        params: logic::frame_fsm::params_for(24.0, 12, true),
        frames: None,
    });
    info!("director: armed default job 24 fps, shutter on, infinite");

    loop {
        match cmds.dequeue() {
            Some(Command::SelfTest) => {
                info!("director: SelfTest — TMC/INDEX/DOOR checks are M3 (stub)");
            }
            Some(Command::Run { fps, frames, shutter }) => {
                info!("director: Run fps={fps} frames={frames:?} shutter={shutter}");
                rt::arm_job(rt::Job {
                    params: logic::frame_fsm::params_for(fps as f32, 12, shutter),
                    frames,
                });
            }
            Some(Command::Stop) => {
                info!("director: Stop — park-at-frame-end arrives in M2c");
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
                // Cooperative poll: the director has this executor to itself,
                // but still yields so interrupts/timers stay happy.
                Timer::after(Duration::from_millis(5)).await;
            }
        }
    }
}
