//! SPECS-derived constants. Single source of truth; the firmware crate re-exports these.

// --- Film / sprocket (SPECS §2) ---
pub const FILM_WIDTH_MM: f32 = 34.98;
pub const PERF_PITCH_MM: f32 = 4.750;
/// Half-perf sprocket: tooth pitch is half the perforation pitch.
pub const TOOTH_PITCH_MM: f32 = PERF_PITCH_MM / 2.0;
pub const SPROCKET_TEETH: u32 = 20;
pub const FULL_STEPS_PER_REV: u32 = 200;
pub const STEPS_PER_TOOTH: u32 = FULL_STEPS_PER_REV / SPROCKET_TEETH;
/// 1.5 perf = 3 half-perfs = 3 teeth.
pub const HALF_PERFS_PER_FRAME: u32 = 3;
pub const FRAME_FULL_STEPS: u32 = HALF_PERFS_PER_FRAME * STEPS_PER_TOOTH;
pub const MICROSTEPS: u32 = 16;
pub const FRAME_USTEPS: u32 = FRAME_FULL_STEPS * MICROSTEPS;
/// Index sensor: one edge per sprocket revolution = 200 full steps.
pub const INDEX_STEPS: u32 = FULL_STEPS_PER_REV;
pub const INDEX_SLACK_STEPS: i32 = 2;

// --- Frame geometry (SPECS §2.1) ---
pub const FRAME_PITCH_MM: f32 = PERF_PITCH_MM * HALF_PERFS_PER_FRAME as f32 / 2.0;
pub const FRAMES_PER_M: f32 = 1000.0 / FRAME_PITCH_MM;

// --- Frame timing (SPECS §3.5) ---
pub const SETTLE_US: u32 = 3_000;

/// Fraction of the move spent in each accel/decel ramp (logic::profile).
pub const PULL_ACCEL_FRAC: f32 = 0.2;
pub const PULLDOWN_WINDOW_PCT: f32 = 55.0;
/// [VERIFY] optional guard between settle end and exposure start (SPECS uses 0).
pub const GUARD_US: u32 = 0;
/// Peak-and-hold: pull-in window before dropping to hold duty (SPECS §3.3).
pub const SHUTTER_PULL_MS: u32 = 4;
pub const SHUTTER_HOLD_PCT: u32 = 25;

// --- FPS / boost (SPECS §4.2) ---
pub const FPS_MIN: f32 = 3.0;
pub const FPS_MAX: f32 = 36.0;
pub const FPS_STEP: f32 = 0.5;
pub const BOOST_MULT: f32 = 1.5;
pub const RAMP_UP_DEFAULT: f32 = 24.0;
pub const RAMP_DOWN_DEFAULT: f32 = 48.0;

// --- Exposure (SPECS §3.5) ---
pub const EXPOSURE_MIN_MS: u32 = 2;
pub const EXPOSURE_MAX_MS: u32 = 1000;

// --- Battery (SPECS §6.2) ---
pub const VBAT_WARN_V: f32 = 19.8;
pub const VBAT_STOP_V: f32 = 18.3;

/// Full steps for any half-perf multiple (variable-perf future: SPECS §2.4).
pub const fn full_steps_for_half_perfs(half_perfs: u32) -> u32 {
    half_perfs * STEPS_PER_TOOTH
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_constants_match_specs() {
        assert_eq!(STEPS_PER_TOOTH, 10);
        assert_eq!(FRAME_FULL_STEPS, 30);
        assert_eq!(FRAME_USTEPS, 480);
        assert_eq!(full_steps_for_half_perfs(2), 20); // 1 perf
        assert_eq!(full_steps_for_half_perfs(4), 40); // 2 perf
        assert_eq!(full_steps_for_half_perfs(8), 80); // 4 perf
    }

    #[test]
    fn frame_geometry() {
        assert!((FRAME_PITCH_MM - 7.125).abs() < 1e-4);
        assert!((FRAMES_PER_M - 140.3).abs() < 0.1);
    }
}
