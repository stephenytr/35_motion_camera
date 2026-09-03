//! TMC2209 transport driver, pin mode (SPECS §7.2, ARCHITECTURE §3.4).
//!
//! Both axes (transport + takeup) are TMC2209 boards driven in **pin mode**:
//! STEP/DIR/EN from the ESP32; microstepping and current are set on the
//! boards (MS1/MS2 jumpers, Vref trimmer) — no comms channel, so there is no
//! telemetry or fault polling. The 16 µstep board setting matches
//! `FRAME_USTEPS` (30 full steps × 16).
//!
//! ENN is active-LOW on the chip (low = enabled). The firmware holds ENN
//! high (disabled/freewheel) at power-on and drives it high again on any
//! safe state; the director enables it after bring-up.

use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::peripherals::{GPIO14, GPIO32};

pub struct Tmc2209 {
    en: Output<'static>,
    dir: Output<'static>,
    /// Level on DIR that corresponds to film-forward (motor wiring may
    /// differ — flip this constant if the transport runs backwards).
    pub forward_level: Level,
}

impl Tmc2209 {
    pub fn new(en: GPIO14<'static>, dir: GPIO32<'static>) -> Self {
        Self {
            // ENN high = disabled: the fail-safe power-on state.
            en: Output::new(en, Level::High, OutputConfig::default()),
            dir: Output::new(dir, Level::High, OutputConfig::default()),
            forward_level: Level::High,
        }
    }

    /// Enable the outputs (called at boot after safe-state setup).
    pub fn enable(&mut self) {
        self.en.set_low();
        #[cfg(feature = "debug-prints")]
        log::info!("motor: transport ENN LOW (enabled), STEP on GPIO15");
    }

    /// Film direction for the next/current job. `true` = forward.
    pub fn set_dir(&mut self, forward: bool) {
        self.dir.set_level(if forward { self.forward_level } else { !self.forward_level });
        #[cfg(feature = "debug-prints")]
        log::info!("motor: transport DIR {} (GPIO32)", if forward { "HIGH (forward)" } else { "LOW (reverse)" });
    }
}
