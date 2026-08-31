//! Frame counters and roll capacity math (SPECS §9.4).

use crate::consts::FRAME_PITCH_MM;

/// Usable frames for a given roll length (floor; a partial frame is unusable).
/// `as u32` truncates = floor for non-negative input and saturates to 0 otherwise
/// (core has no `f32::floor`).
pub fn capacity_frames(roll_length_mm: f32) -> u32 {
    (roll_length_mm / FRAME_PITCH_MM) as u32
}

/// One track's counters. Two-track mode instantiates one per track (SPECS §9.4).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrackCounters {
    pub exposed: u32,
    pub capacity: u32,
}

impl TrackCounters {
    pub fn new(capacity: u32) -> Self {
        Self {
            exposed: 0,
            capacity,
        }
    }

    pub fn remaining(&self) -> u32 {
        self.capacity.saturating_sub(self.exposed)
    }

    /// Returns false when the roll is already full (film-end condition).
    pub fn expose(&mut self) -> bool {
        if self.exposed < self.capacity {
            self.exposed += 1;
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capacity_matches_specs() {
        assert_eq!(capacity_frames(1632.0), 229);
        assert_eq!(capacity_frames(712.5), 100);
    }

    #[test]
    fn track_counters_stop_at_capacity() {
        let mut t = TrackCounters::new(2);
        assert!(t.expose());
        assert!(t.expose());
        assert!(!t.expose());
        assert_eq!(t.remaining(), 0);
    }
}
