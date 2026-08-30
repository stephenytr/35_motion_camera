//! Supervisor task (core 0): applies the interlock matrix (logic::interlock),
//! latches faults, feeds the RTC watchdog, translates UI commands into
//! director queue entries (ARCHITECTURE §5.1–5.3).
//!
//! M1 scope: event-driven loop proving the plumbing — consumes the event
//! channel, mirrors state into `Status`, and holds the CmdProducer half of
//! the command queue for real button/UI input in M4. (Run/Stop/Inch/
//! Rewind/Boost were exercised end-to-end on the bench via a throwaway
//! script before this landed — see the M3b commit message.)

use embassy_time::{Duration, Timer};
use log::info;

use crate::command::CmdProducer;
use crate::fault::{Event, EventChannel};
use crate::status::{State, Status};

#[embassy_executor::task]
pub async fn supervisor_task(
    events: &'static EventChannel,
    _cmds: CmdProducer,
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

    loop {
        match events.receive().await {
            Event::FrameDone(n) => {
                // Silent by design: 24 Hz of FrameDone must not flood the log.
                status.frames_exposed.store(n, core::sync::atomic::Ordering::Relaxed);
            }
            Event::JobComplete => {
                status.set_state(State::Idle);
                info!("supervisor: JobComplete -> IDLE");
            }
            Event::CounterZero => {
                // Rewind-to-zero reached the datum (SPECS §10 step 4).
                status.set_state(State::Idle);
                info!("supervisor: CounterZero — film at datum");
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
            Event::IndexTick => {
                // Sprocket index edge accepted (ARCHITECTURE §4.3) — up to
                // ~3.6 Hz at 24 fps, so no log line; status-only.
            }
            Event::SettingsChanged => info!("supervisor: SettingsChanged"),
        }

        // Keep the cooperative-executor budget honest (ARCHITECTURE §9 rule 6).
        Timer::after(Duration::from_millis(1)).await;
    }
}
