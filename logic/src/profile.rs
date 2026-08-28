//! µstep period tables for the RMT step generator (ARCHITECTURE §3.2).
//!
//! The table gives one period (µs) per microstep. The firmware splits each period
//! into RMT high/low symbol pairs. Invariant tested here: the integral of the table
//! equals the requested pulldown duration EXACTLY — cadence is timer arithmetic and
//! must never drift.

use crate::consts::FRAME_USTEPS;

pub const MAX_USTEPS: usize = FRAME_USTEPS as usize;

#[derive(Debug, Clone, Copy)]
pub struct StepTable {
    pub dt_us: [u32; MAX_USTEPS],
    pub len: usize,
}

impl StepTable {
    pub fn total_us(&self) -> u32 {
        self.dt_us[..self.len].iter().sum()
    }

    pub fn min_dt_us(&self) -> u32 {
        self.dt_us[..self.len].iter().copied().min().unwrap_or(0)
    }
}

/// Symmetric trapezoid profile: velocity ramps linearly over `accel_frac` of the
/// move at each end, plateau between. Integer periods are allocated by largest
/// remainder so the sum is exact.
pub fn build_trapezoid(usteps: usize, duration_us: u32, accel_frac: f32) -> StepTable {
    assert!((1..=MAX_USTEPS).contains(&usteps), "usteps out of range");
    assert!(
        duration_us as usize >= usteps,
        "duration too short for 1 µs/µstep"
    );

    let half = usteps / 2;
    // round(x) for positive x via +0.5 (core has no f32::round)
    let a = ((usteps as f32) * accel_frac + 0.5) as usize;
    let a = a.clamp(1, half.max(1));

    let mut weights = [0.0f32; MAX_USTEPS];
    let mut total_w = 0.0f32;
    for (i, w) in weights.iter_mut().enumerate().take(usteps) {
        let v = if i < a {
            (i + 1) as f32 / a as f32
        } else if i + a >= usteps {
            (usteps - i) as f32 / a as f32
        } else {
            1.0
        };
        *w = 1.0 / v;
        total_w += *w;
    }

    let mut dt = [0u32; MAX_USTEPS];
    let mut fracs = [0.0f32; MAX_USTEPS];
    let mut floored = 0u32;
    for i in 0..usteps {
        let raw = duration_us as f32 * weights[i] / total_w;
        // `as u32` truncates = floor for positive raw (core has no f32::floor)
        dt[i] = raw as u32;
        fracs[i] = raw - dt[i] as f32;
        floored += dt[i];
    }

    let mut rem = (duration_us - floored) as usize;
    while rem > 0 {
        let mut best = 0usize;
        for i in 1..usteps {
            if fracs[i] > fracs[best] {
                best = i;
            }
        }
        dt[best] += 1;
        fracs[best] = -1.0;
        rem -= 1;
    }

    StepTable {
        dt_us: dt,
        len: usteps,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn integral_is_exact() {
        let cases = [
            (1usize, 1u32),
            (2, 3),
            (30, 100_000),
            (480, 4_167),
            (480, 15_277),
            (480, 22_916),
        ];
        for &(n, dur) in &cases {
            let t = build_trapezoid(n, dur, 0.25);
            assert_eq!(t.len, n);
            assert_eq!(t.total_us(), dur, "n={n} dur={dur}");
        }
    }

    #[test]
    fn shape_ramps_up_and_down() {
        let t = build_trapezoid(480, 22_916, 0.25);
        assert!(t.dt_us[0] > t.dt_us[120], "must start slow");
        assert!(t.dt_us[479] > t.dt_us[360], "must end slow");
        assert!(t.min_dt_us() >= 1);
        for &dt in &t.dt_us[..t.len] {
            assert!(dt >= 1);
        }
    }
}
