//! Index-sensor step-loss watchdog (ARCHITECTURE §4.3).
//!
//! Perf counting is open-loop [LOCKED]; the sprocket index edge must arrive every
//! 200 full steps. Frames are 30 steps, so edges land mid-frame except every 20th
//! frame (lcm(30, 200) = 600). The check is arrival-position based: at an edge the
//! cumulative commanded steps must sit within ±INDEX_SLACK_STEPS of a multiple of
//! 200; at each frame end, no edge may be more than 200 + slack steps overdue.

use crate::consts::{INDEX_SLACK_STEPS, INDEX_STEPS};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IndexVerdict {
    Ok,
    /// Signed deviation (steps) from the expected multiple of INDEX_STEPS.
    Misaligned(i32),
    MissedEdge,
}

#[derive(Debug, Default)]
pub struct IndexWatch {
    last_edge_step: i32,
}

impl IndexWatch {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn on_index_edge(&mut self, cumulative_steps: i32) -> IndexVerdict {
        let m = INDEX_STEPS as i32;
        let phase = cumulative_steps.rem_euclid(m);
        let dev = if phase > m / 2 { phase - m } else { phase };
        if dev.abs() > INDEX_SLACK_STEPS {
            return IndexVerdict::Misaligned(dev);
        }
        self.last_edge_step = cumulative_steps;
        IndexVerdict::Ok
    }

    pub fn check_frame_end(&self, cumulative_steps: i32) -> IndexVerdict {
        let overdue = cumulative_steps - self.last_edge_step;
        if overdue > INDEX_STEPS as i32 + INDEX_SLACK_STEPS {
            IndexVerdict::MissedEdge
        } else {
            IndexVerdict::Ok
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nominal_edges_pass() {
        let mut w = IndexWatch::new();
        for k in 1..=10 {
            assert_eq!(w.on_index_edge(200 * k), IndexVerdict::Ok);
        }
    }

    #[test]
    fn edges_at_midframe_positions_pass() {
        // 30-step frames: cumulative at frame ends is 30, 60, ...; edges land at 200k.
        let mut w = IndexWatch::new();
        assert_eq!(w.on_index_edge(200), IndexVerdict::Ok);
        assert_eq!(w.check_frame_end(210), IndexVerdict::Ok);
        assert_eq!(w.on_index_edge(400), IndexVerdict::Ok);
        assert_eq!(w.check_frame_end(600), IndexVerdict::Ok);
    }

    #[test]
    fn drifted_edge_is_misaligned() {
        let mut w = IndexWatch::new();
        assert_eq!(w.on_index_edge(205), IndexVerdict::Misaligned(5));
        assert_eq!(w.on_index_edge(198), IndexVerdict::Ok);
    }

    #[test]
    fn missed_edge_is_detected_at_frame_end() {
        let w = IndexWatch::new();
        assert_eq!(w.check_frame_end(180), IndexVerdict::Ok);
        assert_eq!(w.check_frame_end(210), IndexVerdict::MissedEdge);
    }
}
