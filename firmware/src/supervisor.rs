//! Supervisor task (core 0): applies the interlock matrix (logic::interlock),
//! latches faults, feeds the RTC watchdog, translates UI commands into
//! director queue entries (ARCHITECTURE §5.1–5.3).
//!
//! M1 scope: event-driven loop proving the plumbing — consumes the event
//! channel, mirrors state into `Status`, and sends the boot-time self-test to
//! the director over the command queue.

use embassy_time::{Duration, Timer};
use log::info;

use crate::command::{CmdProducer, Command};
use crate::fault::{Event, EventChannel};
use crate::status::{State, Status};

#[embassy_executor::task]
pub async fn supervisor_task(
    events: &'static EventChannel,
    mut cmds: CmdProducer,
    status: &'static Status,
) {
    status.set_state(State::Idle);
    info!("supervisor: up, state = {:?}", status.state());

    cmds.enqueue(Command::SelfTest).map_err(|_| {
        info!("supervisor: command queue full, dropping SelfTest");
    }).ok();

    loop {
        match events.receive().await {
            Event::FrameDone(n) => {
                status.frames_exposed.store(n, core::sync::atomic::Ordering::Relaxed);
                info!("supervisor: FrameDone({n})");
            }
            Event::JobComplete => {
                status.set_state(State::Idle);
                info!("supervisor: JobComplete -> IDLE");
            }
            Event::DoorOpen => {
                status.door_open.store(true, core::sync::atomic::Ordering::Relaxed);
                status.set_state(State::Door);
                info!("supervisor: DoorOpen -> DOOR");
            }
            Event::DoorClosed => {
                status.door_open.store(false, core::sync::atomic::Ordering::Relaxed);
                info!("supervisor: DoorClosed");
            }
            Event::Fault(code) => {
                status.set_state(State::Error);
                status.fault.store(code.code(), core::sync::atomic::Ordering::Relaxed);
                info!("supervisor: Fault({code:?}) -> ERROR");
            }
            Event::CounterZero => info!("supervisor: CounterZero"),
            Event::IndexTick => info!("supervisor: IndexTick"),
            Event::SettingsChanged => info!("supervisor: SettingsChanged"),
        }

        // Keep the cooperative-executor budget honest (ARCHITECTURE §9 rule 6).
        Timer::after(Duration::from_millis(1)).await;
    }
}
