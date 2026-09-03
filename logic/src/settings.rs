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
        Self::defaults()
    }
}

impl Settings {
    /// `const`-friendly defaults (usable in firmware statics).
    pub const fn defaults() -> Self {
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
            // Ramp rates aren't yet menu-editable, but a CRC-valid record
            // with a corrupt/nonsensical value (0, negative, or absurdly
            // large) should still fall back to defaults rather than load
            // as-is — same reasoning as every other field here.
            && self.ramp_up_fps_s > 0.0
            && self.ramp_up_fps_s <= 200.0
            && self.ramp_down_fps_s > 0.0
            && self.ramp_down_fps_s <= 200.0
    }

    /// Fixed-layout byte form for the storage codec (see
    /// `logic::storage_codec`). Little-endian; the flash payload embeds
    /// exactly these bytes.
    pub fn to_bytes(&self) -> [u8; SETTINGS_BYTES] {
        let mut b = [0u8; SETTINGS_BYTES];
        b[0..2].copy_from_slice(&self.version.to_le_bytes());
        b[2..6].copy_from_slice(&self.fps.to_le_bytes());
        b[6..10].copy_from_slice(&self.exposure_ms.to_le_bytes());
        b[10..14].copy_from_slice(&self.roll_frames.to_le_bytes());
        b[14] = match self.mode {
            TrackMode::Full => 0,
            TrackMode::Dual => 1,
        };
        b[15] = match self.track {
            Track::A => 0,
            Track::B => 1,
        };
        b[16..20].copy_from_slice(&self.boost_mult.to_le_bytes());
        b[20..24].copy_from_slice(&self.ramp_up_fps_s.to_le_bytes());
        b[24..28].copy_from_slice(&self.ramp_down_fps_s.to_le_bytes());
        b[28..32].copy_from_slice(&self.hold_pct.to_le_bytes());
        b
    }

    /// Inverse of `to_bytes`; `None` on an unreadable/corrupt record.
    pub fn from_bytes(b: &[u8; SETTINGS_BYTES]) -> Option<Self> {
        let s = Self {
            version: u16::from_le_bytes([b[0], b[1]]),
            fps: f32::from_le_bytes([b[2], b[3], b[4], b[5]]),
            exposure_ms: u32::from_le_bytes([b[6], b[7], b[8], b[9]]),
            roll_frames: u32::from_le_bytes([b[10], b[11], b[12], b[13]]),
            mode: match b[14] {
                0 => TrackMode::Full,
                1 => TrackMode::Dual,
                _ => return None,
            },
            track: match b[15] {
                0 => Track::A,
                1 => Track::B,
                _ => return None,
            },
            boost_mult: f32::from_le_bytes([b[16], b[17], b[18], b[19]]),
            ramp_up_fps_s: f32::from_le_bytes([b[20], b[21], b[22], b[23]]),
            ramp_down_fps_s: f32::from_le_bytes([b[24], b[25], b[26], b[27]]),
            hold_pct: u32::from_le_bytes([b[28], b[29], b[30], b[31]]),
        };
        if s.is_valid() {
            Some(s)
        } else {
            None
        }
    }
}

pub const SETTINGS_BYTES: usize = 32;

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

    #[test]
    fn corrupted_ramp_rates_are_rejected() {
        let mut s = Settings::default();
        s.ramp_up_fps_s = 0.0;
        assert!(!s.is_valid());
        let mut s = Settings::default();
        s.ramp_down_fps_s = -1.0;
        assert!(!s.is_valid());
        let mut s = Settings::default();
        s.ramp_up_fps_s = 1_000.0;
        assert!(!s.is_valid());
    }

    #[test]
    fn settings_round_trip_bytes() {
        let mut s = Settings::default();
        s.fps = 18.5;
        s.exposure_ms = 45;
        s.roll_frames = 100;
        s.mode = TrackMode::Dual;
        s.track = Track::B;
        s.boost_mult = 1.75;
        assert_eq!(Settings::from_bytes(&s.to_bytes()), Some(s));
    }

    #[test]
    fn corrupt_bytes_are_rejected() {
        let mut b = Settings::default().to_bytes();
        b[14] = 0xFF; // bad mode discriminant
        assert_eq!(Settings::from_bytes(&b), None);
    }
}
