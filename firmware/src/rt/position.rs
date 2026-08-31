//! RT-plane film position (ARCHITECTURE §4.2/§4.3) — the ISR-side shadow of
//! `logic::position`. One atomic µstep accumulator, updated by the heartbeat
//! ISR on every counted frame, direction-aware.
//!
//! Single-writer rule (§9 rule 5): the heartbeat ISR (core 1, P2) is the
//! only writer of POSITION; the director sets DIRECTION at arm time (task
//! context, before the job's first FrameStart — no ISR race), and everyone
//! reads via relaxed loads.

use core::sync::atomic::{AtomicI32, AtomicU8, Ordering};

use logic::position::Direction;

/// Cumulative commanded µsteps from the threading datum (≥ 0).
static POSITION: AtomicI32 = AtomicI32::new(0);

/// Current job direction. 0 = forward, 1 = reverse. Written by the director
/// at arm time; read by the heartbeat ISR.
static DIRECTION: AtomicU8 = AtomicU8::new(0);

pub fn direction() -> Direction {
    match DIRECTION.load(Ordering::Relaxed) {
        1 => Direction::Reverse,
        _ => Direction::Forward,
    }
}

pub fn set_direction(dir: Direction) {
    DIRECTION.store(
        match dir {
            Direction::Forward => 0,
            Direction::Reverse => 1,
        },
        Ordering::Relaxed,
    );
}

/// Advance one frame's worth of film in the current direction (heartbeat ISR
/// at frame-counted time). Returns the new position in µsteps.
pub fn advance_frame(dir: Direction) -> i32 {
    let delta = dir.sign() * logic::consts::FRAME_USTEPS as i32;
    let mut p = POSITION.load(Ordering::Relaxed);
    loop {
        let next = (p + delta).max(0);
        match POSITION.compare_exchange_weak(p, next, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => return next,
            Err(actual) => p = actual,
        }
    }
}

pub fn usteps() -> i32 {
    POSITION.load(Ordering::Relaxed)
}

pub fn frames() -> u32 {
    (usteps() / logic::consts::FRAME_USTEPS as i32) as u32
}

pub fn at_datum() -> bool {
    usteps() == 0
}

/// Reset the datum (re-threading, SPECS §10 steps 2/5). Bench scripts use
/// this between procedures until the UI's threading flow lands.
#[allow(dead_code)]
pub fn reset() {
    POSITION.store(0, Ordering::Relaxed);
}
