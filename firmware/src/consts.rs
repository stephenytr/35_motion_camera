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

// NOTE: the HIL timing debug strobe (ARCHITECTURE §11/§13, decision log
// #16/#19) previously had a placeholder `DEBUG_STROBE_GPIO = 27` constant
// here. Decision #32's M4c hardware swap put the shutter MOSFET gate on
// GPIO27 (`shutter_pins::GATE`, below) and nobody updated the strobe
// constant — it was unwired dead code so there was no *live* conflict, but
// anyone following §11's bring-up instructions to "scope the debug strobe
// pins" at GPIO27 would have scoped (and, if the strobe were ever wired up
// as documented, toggled) the shutter solenoid gate instead. Removed until
// a real free GPIO is picked for it.

/// TMC2209 × 2 bench pin map (decision log #32): both drivers run in pin
/// mode (STEP/DIR/EN; MS jumpers + Vref on the boards). S3 port (decision
/// log #33): GPIO22-25 don't exist on the ESP32-S3 and GPIO33-37 are octal
/// PSRAM on the N16R8 module, so the takeup STEP moved 23 → 38; transport
/// pins (14/15/32) are free on the S3 and stay put.
///
/// TAKEUP_EN was originally GPIO12 — moved to GPIO13. GPIO12 (MTDI) is an
/// ESP32 boot strapping pin that selects flash voltage (VDD_SDIO) at
/// reset; most TMC2209 breakout boards pull EN high by default (fail-safe
/// disabled state), which forced GPIO12 high on every reset and made the
/// bootloader mis-select 1.8V flash against this board's 3.3V flash,
/// corrupting every SPI read (`invalid header: 0xffffffff`) until the RTC
/// WDT fired again — a multi-cycle boot-loop that looked like "the whole
/// system halts" after any unrelated reset during a run. GPIO13 is not a
/// strapping pin on either chip.
#[allow(dead_code)]
pub mod tmc2209_pins {
    /// ENN — active LOW: low = driver enabled, high = disabled/freewheel.
    pub const TRANSPORT_EN: u8 = 14;
    pub const TRANSPORT_STEP: u8 = 15;
    pub const TRANSPORT_DIR: u8 = 32;
    pub const TAKEUP_EN: u8 = 13;
    /// Takeup STEP is an LEDC pulse train (LS channel 1 on the S3).
    pub const TAKEUP_STEP: u8 = 38;
    pub const TAKEUP_DIR: u8 = 21;
}

/// Bench UI pin map (decision log #29/#31/#33): SSD1306 OLED on I2C0 +
/// seven buttons + two pots. S3 moves: SCL off GPIO19 (USB D- on devkits)
/// to 17; MENU/▲/▼ off 25/26/22 (nonexistent/PSRAM) to 26/29/28; BOOST off
/// 33 (octal PSRAM) to 48; FRAME off 36 (octal PSRAM) to 12; pots off
/// 34/35 (not ADC pins on the S3) to GPIO1/2 = ADC1_CH0/1. All buttons
/// active-low with internal pull-ups (the S3 supports pull-ups on every
/// GPIO — the classic-ESP32 external-pull-up requirement is retired).
#[allow(dead_code)]
pub mod ui_pins {
    pub const OLED_SDA: u8 = 18;
    pub const OLED_SCL: u8 = 17;
    pub const RUN: u8 = 5;
    pub const MENU: u8 = 26;
    pub const UP: u8 = 29;
    pub const DOWN: u8 = 28;
    /// Shooting cluster (SPECS §9.3).
    pub const BOOST: u8 = 48;
    pub const FRAME: u8 = 12;
    pub const INCH: u8 = 39;
    /// Pots (decision log #30): B10K dividers on ADC1.
    pub const POT_FPS: u8 = 1;
    pub const POT_EXPOSURE: u8 = 2;
}

/// Shutter MOSFET gate (drivers::shutter): LEDC high-speed channel 0
/// (20 kHz peak-and-hold waveform), 10k gate pulldown on the bench.
#[allow(dead_code)]
pub mod shutter_pins {
    pub const GATE: u8 = 27;
}
