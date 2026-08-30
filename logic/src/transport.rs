//! Transport procedure model (SPECS §10) — pure logic.
//!
//! The two-track leader-marking procedure is procedural bookkeeping over the
//! `Position` accumulator: leader marks (6 exposed frames), a leader gap
//! (10 blind-advance frames), shooting, rewind-to-zero, and track-B setup.
//! The director keeps one of these per job chain; the ISRs never see it.

use crate::position::{Direction, Position};

/// Leader marks: dense exposed frames at a bright target (SPECS §10 step 3).
pub const LEADER_MARK_FRAMES: u32 = 6;
/// Leader gap: blind advance after the marks (lens capped), the inspection
/// zone (SPECS §10 step 3).
pub const LEADER_GAP_FRAMES: u32 = 10;
/// Frames consumed by the whole leader procedure.
pub const LEADER_TOTAL_FRAMES: u32 = LEADER_MARK_FRAMES + LEADER_GAP_FRAMES;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum ScriptPhase {
    /// Not in a script (normal run/stop/inch/rewind commands).
    #[default]
    Idle,
    /// Leader marks job (6 frames, shutter on) is running.
    LeaderMarks,
    /// Leader gap job (10 frames, shutter off) is running.
    LeaderGap,
    /// Track-B setup advance is running.
    TrackBAdvance,
}

/// The director's view of the film procedure. Pure state, no hardware.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Transport {
    position: Position,
    /// Position (in whole frames) at the end of the last completed pass.
    /// Updated at every job-complete boundary; Track-B setup advances from
    /// the datum to this position to re-align pass 2 with pass 1 (SPECS §10
    /// step 6: leader + gap + frames-used-on-A).
    pass_end_frames: u32,
    phase: ScriptPhase,
}

impl Transport {
    pub const fn new() -> Self {
        Self {
            position: Position::new(),
            pass_end_frames: 0,
            phase: ScriptPhase::Idle,
        }
    }

    pub fn position_frames(&self) -> u32 {
        self.position.frames()
    }

    pub fn at_datum(&self) -> bool {
        self.position.at_datum()
    }

    pub const fn phase(&self) -> ScriptPhase {
        self.phase
    }

    pub fn set_phase(&mut self, phase: ScriptPhase) {
        self.phase = phase;
    }

    /// Record one completed frame of motion (called per counted frame).
    pub fn on_frame(&mut self, dir: Direction) {
        self.position.advance_frame(dir);
    }

    /// Pull the position from the RT-plane accumulator (the director calls
    /// this at job boundaries — the ISR owns the real counter, this model
    /// is the command-plane shadow).
    pub fn sync_position(&mut self, usteps: i32) {
        self.position = Position::from_usteps(usteps);
    }

    /// A job ended (JobComplete / park). The film is at rest here, so this
    /// is the "end of pass" datum for track-B re-alignment — but only for
    /// *forward* motion: a rewind-end (back at the datum) must not clobber
    /// the pass-end position recorded when the shooting pass stopped.
    pub fn on_job_end(&mut self, dir: Direction) {
        if dir == Direction::Forward {
            self.pass_end_frames = self.position.frames();
        }
        self.phase = ScriptPhase::Idle;
    }

    /// Frames to rewind to reach the datum (Rewind to zero, SPECS §10 step 4).
    pub fn rewind_plan(&self) -> u32 {
        self.position.frames()
    }

    /// Frames to advance for track-B setup (SPECS §10 step 6).
    pub fn track_b_plan(&self) -> u32 {
        self.pass_end_frames
    }

    /// Reset the position datum (used after manual re-threading; SPECS §10
    /// steps 2 and 5 treat the new datum as position zero).
    pub fn reset_datum(&mut self) {
        self.position = Position::new();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run_frames(t: &mut Transport, dir: Direction, n: u32) {
        for _ in 0..n {
            t.on_frame(dir);
        }
    }

    #[test]
    fn leader_procedure_consumes_16_frames() {
        let mut t = Transport::new();
        t.set_phase(ScriptPhase::LeaderMarks);
        run_frames(&mut t, Direction::Forward, LEADER_MARK_FRAMES);
        t.on_job_end(Direction::Forward);
        assert_eq!(t.pass_end_frames, LEADER_MARK_FRAMES);

        t.set_phase(ScriptPhase::LeaderGap);
        run_frames(&mut t, Direction::Forward, LEADER_GAP_FRAMES);
        t.on_job_end(Direction::Forward);
        assert_eq!(t.pass_end_frames, LEADER_TOTAL_FRAMES);
    }

    #[test]
    fn rewind_plan_returns_remaining_frames() {
        let mut t = Transport::new();
        run_frames(&mut t, Direction::Forward, 24);
        assert_eq!(t.rewind_plan(), 24);
        run_frames(&mut t, Direction::Reverse, 24);
        assert!(t.at_datum());
        assert_eq!(t.rewind_plan(), 0);
    }

    #[test]
    fn track_b_plan_matches_pass_end() {
        let mut t = Transport::new();
        run_frames(&mut t, Direction::Forward, LEADER_TOTAL_FRAMES);
        run_frames(&mut t, Direction::Forward, 40); // track A shots
        t.on_job_end(Direction::Forward);
        assert_eq!(t.pass_end_frames, LEADER_TOTAL_FRAMES + 40);
        // Rewind to the datum, then track-B setup must advance the same
        // total to re-align pass 2 with pass 1.
        let plan = t.track_b_plan();
        let to_rewind = t.rewind_plan();
        run_frames(&mut t, Direction::Reverse, to_rewind);
        assert!(t.at_datum());
        assert_eq!(plan, LEADER_TOTAL_FRAMES + 40);
        t.set_phase(ScriptPhase::TrackBAdvance);
        run_frames(&mut t, Direction::Forward, plan);
        assert_eq!(t.position_frames(), plan);
        t.on_job_end(Direction::Forward);
        assert_eq!(t.phase(), ScriptPhase::Idle);
    }

    #[test]
    fn reset_datum_zeroes_position() {
        let mut t = Transport::new();
        run_frames(&mut t, Direction::Forward, 10);
        t.reset_datum();
        assert!(t.at_datum());
        assert_eq!(t.position_frames(), 0);
    }
}
