//! Boost FPS ramp (SPECS §4.2, ARCHITECTURE §6).
//!
//! Runs in the director task where f32 is allowed; the resulting fps is converted
//! to integer µs phase params at the frame boundary.

use crate::consts::{BOOST_MULT, FPS_MAX, FPS_MIN};

pub struct BoostRamp {
    base: f32,
    current: f32,
    target: f32,
    rate_up: f32,
    rate_down: f32,
    boosting: bool,
}

impl BoostRamp {
    pub fn new(base_fps: f32, rate_up_fps_s: f32, rate_down_fps_s: f32) -> Self {
        let base = base_fps.clamp(FPS_MIN, FPS_MAX);
        Self {
            base,
            current: base,
            target: base,
            rate_up: rate_up_fps_s,
            rate_down: rate_down_fps_s,
            boosting: false,
        }
    }

    pub fn boost_on(&mut self) {
        self.boosting = true;
        self.target = (self.base * BOOST_MULT).clamp(FPS_MIN, FPS_MAX);
    }

    pub fn boost_off(&mut self) {
        self.boosting = false;
        self.target = self.base;
    }

    pub fn set_base(&mut self, fps: f32) {
        self.base = fps.clamp(FPS_MIN, FPS_MAX);
        self.target = if self.boosting {
            (self.base * BOOST_MULT).clamp(FPS_MIN, FPS_MAX)
        } else {
            self.base
        };
    }

    /// Advance by `dt_s` seconds; returns the effective fps for the next frame.
    pub fn step(&mut self, dt_s: f32) -> f32 {
        let rate = if self.current < self.target {
            self.rate_up
        } else {
            self.rate_down
        };
        let delta = rate * dt_s;
        if (self.target - self.current).abs() <= delta {
            self.current = self.target;
        } else if self.current < self.target {
            self.current += delta;
        } else {
            self.current -= delta;
        }
        self.current
    }

    pub fn current(&self) -> f32 {
        self.current
    }

    pub fn at_target(&self) -> bool {
        (self.current - self.target).abs() < f32::EPSILON
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boost_ramp_up_then_down() {
        let mut r = BoostRamp::new(24.0, 24.0, 48.0);
        r.boost_on();
        assert_eq!(r.step(0.25), 30.0);
        assert_eq!(r.step(0.25), 36.0);
        assert!(r.at_target());
        r.boost_off();
        assert_eq!(r.step(0.25), 24.0);
    }

    #[test]
    fn boost_target_is_clamped_to_fps_max() {
        let mut r = BoostRamp::new(30.0, 24.0, 48.0);
        r.boost_on();
        assert_eq!(r.step(10.0), FPS_MAX);
    }
}
