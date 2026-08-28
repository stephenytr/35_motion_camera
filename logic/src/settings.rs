//! Persisted settings model (SPECS §9.4).
//!
//! The flash codec (versioned record + CRC32, ping-pong 4 KB sectors) is the
//! firmware storage task's job; this module only defines and validates the model.

use crate::consts::{
    BOOST_MULT, EXPOSURE_MAX_MS, EXPOSURE_MIN_MS, FPS_MAX, FPS_MIN, RAMP_DOWN_DEFAULT,
    RAMP_UP_DEFAULT, SHUTTER_HOLD_PCT,
};

pub const SETTINGS_VERSION: u16 = 1;

/// 36-exp cartridge ≈ 1.63 m → 228 usable frames at 1.5 perf (SPECS §9.4 quotes ≈229).
pub const ROLL_PRESET_36EXP_MM: f32 = 1630.0;
pub const ROLL_PRESET_24EXP_MM: f32 = 1090.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackMode {
    Full,
    Dual,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Track {
    A,
    B,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    pub version: u16,
    pub fps: f32,
    pub exposure_ms: u32,
    pub roll_frames: u32,
    pub mode: TrackMode,
    pub track: Track,
    pub boost_mult: f32,
    pub ramp_up_fps_s: f32,
    pub ramp_down_fps_s: f32,
    pub hold_pct: u32,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            version: SETTINGS_VERSION,
            fps: 24.0,
            exposure_ms: 12,
            roll_frames: 228,
            mode: TrackMode::Full,
            track: Track::A,
            boost_mult: BOOST_MULT,
            ramp_up_fps_s: RAMP_UP_DEFAULT,
            ramp_down_fps_s: RAMP_DOWN_DEFAULT,
            hold_pct: SHUTTER_HOLD_PCT,
        }
    }
}

impl Settings {
    pub fn is_valid(&self) -> bool {
        self.version == SETTINGS_VERSION
            && self.fps >= FPS_MIN
            && self.fps <= FPS_MAX
            && self.exposure_ms >= EXPOSURE_MIN_MS
            && self.exposure_ms <= EXPOSURE_MAX_MS
            && self.boost_mult >= 1.0
            && self.boost_mult <= 2.0
            && self.hold_pct <= 100
            && self.roll_frames > 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_are_valid() {
        assert!(Settings::default().is_valid());
    }

    #[test]
    fn corrupted_settings_are_rejected() {
        let mut s = Settings::default();
        s.version = 0;
        assert!(!s.is_valid());
        let mut s = Settings::default();
        s.fps = 99.0;
        assert!(!s.is_valid());
        let mut s = Settings::default();
        s.exposure_ms = 0;
        assert!(!s.is_valid());
    }
}
