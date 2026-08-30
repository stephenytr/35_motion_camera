//! Film position accounting (ARCHITECTURE §4.2, §4.3) — pure logic.
//!
//! The RT plane tracks cumulative commanded position in µsteps from the
//! frame-zero datum (threading position). One frame of film is exactly
//! `FRAME_USTEPS` (480) µsteps in either direction, so whole-frame bookkeeping
//! is exact: position is always a multiple of 480, and `frames()` is exact
//! division. Rewind-to-zero parks when the position returns to the datum;
//! the index watchdog (§4.3) consumes the same accumulator.

use crate::consts::FRAME_USTEPS;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Reverse,
}

impl Direction {
    pub const fn sign(self) -> i32 {
        match self {
            Direction::Forward => 1,
            Direction::Reverse => -1,
        }
    }

    pub const fn flipped(self) -> Self {
        match self {
            Direction::Forward => Direction::Reverse,
            Direction::Reverse => Direction::Forward,
        }
    }
}

/// Cumulative position in µsteps from the threading datum. Never negative:
/// reverse motion clamps at the datum (rewind-to-zero's stop condition).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Position {
    usteps: i32,
}

impl Position {
    pub const fn new() -> Self {
        Self { usteps: 0 }
    }

    pub const fn from_usteps(usteps: i32) -> Self {
        Self {
            usteps: if usteps < 0 { 0 } else { usteps },
        }
    }

    pub const fn usteps(&self) -> i32 {
        self.usteps
    }

    /// Whole frames from the datum. Exact: position is always a multiple of
    /// `FRAME_USTEPS` by construction.
    pub fn frames(&self) -> u32 {
        (self.usteps / FRAME_USTEPS as i32) as u32
    }

    /// At the threading datum (rewind-to-zero reached).
    pub fn at_datum(&self) -> bool {
        self.usteps == 0
    }

    /// Advance one frame's worth of film in `dir`; clamps at the datum when
    /// reversing past it. Returns the new position in µsteps.
    pub fn advance_frame(&mut self, dir: Direction) -> i32 {
        self.usteps += dir.sign() * FRAME_USTEPS as i32;
        if self.usteps < 0 {
            self.usteps = 0;
        }
        self.usteps
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn forward_and_reverse_are_exact() {
        let mut p = Position::new();
        for _ in 0..10 {
            p.advance_frame(Direction::Forward);
        }
        assert_eq!(p.frames(), 10);
        assert_eq!(p.usteps(), 10 * FRAME_USTEPS as i32);
        for _ in 0..10 {
            p.advance_frame(Direction::Reverse);
        }
        assert!(p.at_datum());
        assert_eq!(p.frames(), 0);
    }

    #[test]
    fn reverse_clamps_at_datum() {
        let mut p = Position::new();
        p.advance_frame(Direction::Forward);
        p.advance_frame(Direction::Reverse);
        p.advance_frame(Direction::Reverse);
        p.advance_frame(Direction::Reverse);
        assert!(p.at_datum());
        assert_eq!(p.usteps(), 0);
    }

    #[test]
    fn direction_signs() {
        assert_eq!(Direction::Forward.sign(), 1);
        assert_eq!(Direction::Reverse.sign(), -1);
        assert_eq!(Direction::Forward.flipped(), Direction::Reverse);
    }
}
