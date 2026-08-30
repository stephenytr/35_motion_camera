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
/// Bench: user LED, active-high marks the frame phase. Moved from GPIO 13 in
/// M3 — 13 is the TMC SPI MISO on the bench. Documentation only; main.rs
/// wires the typed `peripherals.GPIO27` pin directly.
#[allow(dead_code)]
pub const DEBUG_STROBE_GPIO: u8 = 27;

/// TMC2240/5160 bench pin map (decision log #19). Deviates from the SPECS
/// HIL map (CS=10, MOSI=11, MISO=13, EN=14, STEP=15, DIR=16) because a
/// WROVER devkit's internal flash/PSRAM consumes GPIO 6-11 and 16-17.
/// Documented here (not type-level: pins are compile-time GPIO types in
/// main.rs).
#[allow(dead_code)]
pub mod tmc_pins {
    pub const CS: u8 = 21;
    pub const MOSI: u8 = 23;
    pub const MISO: u8 = 13;
    pub const SCK: u8 = 12;
    /// ENN — active LOW: low = driver enabled, high = disabled/freewheel.
    pub const EN: u8 = 14;
    pub const STEP: u8 = 15;
    pub const DIR: u8 = 32;
}

/// Bench UI pin map (decision log #29): 1602A LCD on I2C0 + four buttons.
/// SDA/SCL are 18/19 because the chip-default 21/22 pair is taken (21 =
/// TMC CS). All buttons active-low with internal pull-ups.
#[allow(dead_code)]
pub mod ui_pins {
    pub const LCD_SDA: u8 = 18;
    pub const LCD_SCL: u8 = 19;
    pub const RUN: u8 = 5;
    pub const MENU: u8 = 25;
    pub const UP: u8 = 26;
    pub const DOWN: u8 = 22;
}
