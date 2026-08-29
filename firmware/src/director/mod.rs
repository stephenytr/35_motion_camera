//! Motion director — the only task on core 1 (ARCHITECTURE §6).
//!
//! M1 scope: drains the command queue on core 1 and acknowledges jobs back to
//! the command plane over the event channel, proving both inter-plane links.
//!
//! TODO(M3+): translate commands into jobs (logic::frame_fsm params), own
//! TMC5160 over SPI2 (init, currents, 250 ms status polls), run the boost
//! ramp (logic::ramp), rebuild RMT step tables (logic::profile), program
//! integer-µs phase params at frame boundaries, brownout Recover job.

use embassy_time::{Duration, Timer};
use log::info;

use crate::command::{CmdConsumer, Command};
use crate::fault::{Event, EventChannel};
use crate::status::Status;

#[embassy_executor::task]
pub async fn director_task(
    events: &'static EventChannel,
    mut cmds: CmdConsumer,
    _status: &'static Status,
) {
    info!(
        "director: up on core {}",
        esp_hal::system::Cpu::current() as usize
    );

    loop {
        match cmds.dequeue() {
            Some(Command::SelfTest) => {
                info!("director: SelfTest — TMC/INDEX/DOOR checks are M3 (stub ok)");
                let _ = events.try_send(Event::JobComplete);
            }
            Some(Command::Run { fps, frames, shutter }) => {
                info!("director: Run fps={fps} frames={frames:?} shutter={shutter}");
                // M2+ runs the real job; for M1 the job completes immediately.
                let _ = events.try_send(Event::JobComplete);
            }
            Some(Command::Stop) => {
                info!("director: Stop");
                let _ = events.try_send(Event::JobComplete);
            }
            Some(Command::Inch { frames }) => {
                info!("director: Inch {frames} frame(s)");
                let _ = events.try_send(Event::JobComplete);
            }
            Some(Command::Rewind { to_zero }) => {
                info!("director: Rewind to_zero={to_zero}");
                let _ = events.try_send(Event::JobComplete);
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
