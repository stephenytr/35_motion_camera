//! SPECS/ARCHITECTURE constants plus firmware-only additions.


/// Xtensa interrupt priorities (esp_hal::interrupt::Priority) — ARCHITECTURE §2.
#[allow(dead_code)]
pub mod isr_priority {
    /// P3: deadman timer + door interlock (last-line safety).
    pub const SAFETY: u8 = 3;
    /// P2: heartbeat, exposure/peak-hold timers, index GPIO.
    pub const FRAME_TIMING: u8 = 2;
    /// P1: embassy executors and all async drivers (defaults).
    pub const ASYNC: u8 = 1;
}

/// HIL timing debug strobe pin (ARCHITECTURE §11): frame strobe + phase marker.
/// Documentation only; main.rs wires the typed pin directly.
#[allow(dead_code)]
pub const DEBUG_STROBE_GPIO: u8 = 27;

/// TMC2209 × 2 bench pin map (decision log #32): both drivers run in pin
/// mode (STEP/DIR/EN; MS jumpers + Vref on the boards). The transport axis
/// keeps the old 2240 STEP/DIR/EN pins (15/32/14); the takeup reuses the
/// pins the retired SPI interface freed (23/21/12). A WROVER devkit's
/// internal flash/PSRAM consumes GPIO 6-11 and 16-17. Documented here
/// (not type-level: pins are compile-time GPIO types in main.rs).
#[allow(dead_code)]
pub mod tmc2209_pins {
    /// ENN — active LOW: low = driver enabled, high = disabled/freewheel.
    pub const TRANSPORT_EN: u8 = 14;
    pub const TRANSPORT_STEP: u8 = 15;
    pub const TRANSPORT_DIR: u8 = 32;
    pub const TAKEUP_EN: u8 = 12;
    /// Takeup STEP is an LEDC pulse train (channel 1), not RMT.
    pub const TAKEUP_STEP: u8 = 23;
    pub const TAKEUP_DIR: u8 = 21;
}

/// Bench UI pin map (decision log #29/#31): SSD1306 OLED on I2C0 + seven
/// buttons + two pots. SDA/SCL are 18/19 (the chip-default 21/22 pair is
/// taken by the takeup DIR / ▼). All buttons active-low with internal
/// pull-ups except the input-only 36/39.
#[allow(dead_code)]
pub mod ui_pins {
    pub const OLED_SDA: u8 = 18;
    pub const OLED_SCL: u8 = 19;
    pub const RUN: u8 = 5;
    pub const MENU: u8 = 25;
    pub const UP: u8 = 26;
    pub const DOWN: u8 = 22;
    /// Shooting cluster (SPECS §9.3). 33 has an internal pull-up; 36/39
    /// are input-only and need external 10k pull-ups on the bench.
    pub const BOOST: u8 = 33;
    pub const FRAME: u8 = 36;
    pub const INCH: u8 = 39;
    /// Pots (decision log #30): B10K dividers on ADC1.
    pub const POT_FPS: u8 = 34;
    pub const POT_EXPOSURE: u8 = 35;
}

/// Shutter MOSFET gate (drivers::shutter): LEDC high-speed channel 0
/// (20 kHz peak-and-hold waveform), 10k gate pulldown on the bench.
#[allow(dead_code)]
pub mod shutter_pins {
    pub const GATE: u8 = 27;
}
