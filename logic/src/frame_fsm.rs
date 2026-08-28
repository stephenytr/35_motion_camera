//! Frame-phase FSM: pure logic executed by the heartbeat ISR (ARCHITECTURE §3.1, §4.1).

use crate::consts::{
    EXPOSURE_MAX_MS, EXPOSURE_MIN_MS, FPS_MAX, FPS_MIN, GUARD_US, PULLDOWN_WINDOW_PCT, SETTLE_US,
    SHUTTER_PULL_MS,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    FrameStart,
    ExposeStart,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameParams {
    pub period_us: u32,
    pub pull_us: u32,
    pub settle_us: u32,
    pub exp_us: u32,
    pub shutter_enabled: bool,
}

/// Register-level actions the ISR wrapper applies (ARCHITECTURE §4.1).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Actions {
    pub rmt_kick: bool,
    pub arm_heartbeat_us: Option<u32>,
    pub shutter_pull: bool,
    pub arm_hold_us: Option<u32>,
    pub arm_exposure_us: Option<u32>,
    pub frame_counted: bool,
    pub park: bool,
}

pub fn period_us(fps: f32) -> u32 {
    (1_000_000.0 / fps) as u32
}

pub fn pull_us(period_us: u32) -> u32 {
    (period_us as f32 * PULLDOWN_WINDOW_PCT / 100.0) as u32
}

pub fn max_exposure_us(period: u32, pull: u32, settle: u32) -> u32 {
    period - pull - settle - GUARD_US
}

pub fn clamp_exposure_ms(requested_ms: u32, period: u32, pull: u32, settle: u32) -> u32 {
    let req = (requested_ms * 1000).clamp(EXPOSURE_MIN_MS * 1000, EXPOSURE_MAX_MS * 1000);
    req.min(max_exposure_us(period, pull, settle))
}

/// Build valid params from user settings; clamps fps and exposure (SPECS §3.5 table).
pub fn params_for(fps: f32, exposure_ms: u32, shutter_enabled: bool) -> FrameParams {
    let fps = fps.clamp(FPS_MIN, FPS_MAX);
    let period = period_us(fps);
    let pull = pull_us(period);
    FrameParams {
        period_us: period,
        pull_us: pull,
        settle_us: SETTLE_US,
        exp_us: clamp_exposure_ms(exposure_ms, period, pull, SETTLE_US),
        shutter_enabled,
    }
}

/// Advance the frame FSM one phase. `remaining` is decremented when a frame is
/// exposed; `None` means run until stopped.
pub fn advance(phase: Phase, p: &FrameParams, remaining: &mut Option<u32>) -> Actions {
    match phase {
        Phase::FrameStart => {
            if *remaining == Some(0) {
                Actions {
                    park: true,
                    ..Default::default()
                }
            } else {
                Actions {
                    rmt_kick: true,
                    arm_heartbeat_us: Some(p.pull_us + p.settle_us),
                    ..Default::default()
                }
            }
        }
        Phase::ExposeStart => {
            if let Some(r) = remaining {
                *r = r.saturating_sub(1);
            }
            let mut a = Actions {
                frame_counted: true,
                arm_heartbeat_us: Some(p.period_us - p.pull_us - p.settle_us),
                ..Default::default()
            };
            if p.shutter_enabled {
                a.shutter_pull = true;
                a.arm_hold_us = Some(SHUTTER_PULL_MS * 1000);
                a.arm_exposure_us = Some(p.exp_us);
            }
            a
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exposure_clamp_matches_specs_table() {
        let p24 = params_for(24.0, 1000, true);
        assert_eq!(p24.period_us, 41_666);
        assert_eq!(p24.pull_us, 22_916);
        assert_eq!(p24.exp_us, 15_750); // SPECS §3.5: ~15.7 ms @ 24 fps

        let p36 = params_for(36.0, 1000, true);
        assert_eq!(p36.exp_us, 9_500); // SPECS §3.5: ~9.5 ms @ 36 fps

        let p3 = params_for(3.0, 1000, true);
        assert_eq!(p3.exp_us, 147_000);

        let p12 = params_for(12.0, 1000, true);
        assert_eq!(p12.exp_us, 34_500);
    }

    #[test]
    fn fps_is_clamped() {
        assert_eq!(params_for(99.0, 10, true).period_us, period_us(FPS_MAX));
        assert_eq!(params_for(0.1, 10, true).period_us, period_us(FPS_MIN));
    }

    #[test]
    fn single_frame_parks_after_one_cycle() {
        let p = params_for(24.0, 12, true);
        let mut remaining = Some(1u32);

        let a = advance(Phase::FrameStart, &p, &mut remaining);
        assert!(a.rmt_kick);
        assert_eq!(a.arm_heartbeat_us, Some(p.pull_us + p.settle_us));

        let a = advance(Phase::ExposeStart, &p, &mut remaining);
        assert!(a.frame_counted);
        assert!(a.shutter_pull);
        assert!(a.arm_exposure_us.is_some());
        assert_eq!(remaining, Some(0));

        let a = advance(Phase::FrameStart, &p, &mut remaining);
        assert!(a.park);
        assert!(!a.rmt_kick);
    }

    #[test]
    fn infinite_job_never_parks() {
        let p = params_for(24.0, 12, true);
        let mut remaining: Option<u32> = None;
        for _ in 0..100 {
            assert!(!advance(Phase::FrameStart, &p, &mut remaining).park);
            assert!(!advance(Phase::ExposeStart, &p, &mut remaining).park);
        }
        assert_eq!(remaining, None);
    }

    #[test]
    fn shutterless_job_has_no_shutter_actions() {
        let p = params_for(24.0, 12, false);
        let mut remaining = Some(5u32);
        let a = advance(Phase::ExposeStart, &p, &mut remaining);
        assert!(a.frame_counted);
        assert!(!a.shutter_pull);
        assert_eq!(a.arm_hold_us, None);
        assert_eq!(a.arm_exposure_us, None);
    }
}
