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
/// Commands are *constructed* by the UI layer (buttons/menu, M4) and by
/// bring-up bench scripts; the director only consumes them. Until M4 lands
/// the enum looks dead to the compiler — allow it explicitly.
#[allow(dead_code)]
pub enum Command {
    SelfTest,
    Run { fps: u8, frames: Option<u32>, shutter: bool },
    Stop,
    Inch { frames: u32 },
    Rewind { to_zero: bool },
    /// SPECS §10 step 3: leader marks (6 frames @6 fps, shutter) followed by
    /// a 10-frame blind gap, then park — a two-job script chain.
    LeaderMark,
    /// SPECS §10 step 6: advance to the recorded pass-end position at creep
    /// speed, aligning track B with pass 1.
    TrackBSetup,
    /// TEMP bench hooks for the index watchdog (no sensor wired yet). The
    /// director executes them on core 1 — index state is CS-mutex-guarded
    /// core-1 data, so the bench script must not touch it directly from
    /// core 0. Removed once the index-sensor ISR + boot self-test land.
    IndexArm,
    IndexEdgeAt(u32),
    SetFps(u8),
    SetExposure(u32),
    Boost(bool),
}

pub type CmdQueue = Queue<Command, 8>;
pub type CmdProducer = Producer<'static, Command>;
pub type CmdConsumer = Consumer<'static, Command>;

/// Boot-time storage for the queue; split into producer/consumer in `main`.
pub static CMD_QUEUE_CELL: StaticCell<CmdQueue> = StaticCell::new();
