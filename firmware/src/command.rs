//! Inter-plane command queue: supervisor (core 0) → director (core 1).
//!
//! ARCHITECTURE §5.2: single producer × single consumer, genuinely lock-free
//! (`heapless::spsc::Queue` uses atomic indices), 8 entries deep.
//!
//! The queue is split once at boot; each half is owned by exactly one task,
//! so no `Sync` is needed and there is never a lock.

use heapless::spsc::{Consumer, Producer, Queue};
use static_cell::StaticCell;

/// Commands the supervisor translates from UI inputs (ARCHITECTURE §5.2).
/// Mirrors the jobs model §4.2 plus settings/self-test.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    SelfTest,
    Run { fps: u8, frames: Option<u32>, shutter: bool },
    Stop,
    Inch { frames: u32 },
    Rewind { to_zero: bool },
    SetFps(u8),
    SetExposure(u32),
    Boost(bool),
}

pub type CmdQueue = Queue<Command, 8>;
pub type CmdProducer = Producer<'static, Command>;
pub type CmdConsumer = Consumer<'static, Command>;

/// Boot-time storage for the queue; split into producer/consumer in `main`.
pub static CMD_QUEUE_CELL: StaticCell<CmdQueue> = StaticCell::new();
