//! SSD1306 128×64 OLED over I2C (SPECS §9.1) — buffered graphics mode with
//! the embedded-graphics 0.8 `FONT_8X13` ASCII font: 16 columns × 8 px fills
//! the 128 px width exactly, matching the UI's two 16-wide lines.
//!
//! Address 0x3C (standard) or 0x3D (alternate), auto-detected at boot by a
//! raw ACK probe. The driver owns the I2C bus (same pattern as the retired
//! bench LCD1602); a missing display leaves the UI headless.

use embedded_graphics::mono_font::ascii::FONT_8X13;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use esp_hal::i2c::master::I2c;
use esp_hal::Blocking;
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::prelude::*;
use ssd1306::{I2CDisplayInterface, Ssd1306};

/// Columns per line (8 px/char × 16 = 128 px).
pub const COLS: u8 = 16;
/// Number of text lines (4 × 13 px rows fit the 64 px height).
pub const ROWS: usize = 4;
/// Line baselines for FONT_8X13 (height 13 px).
const LINE_YS: [i32; ROWS] = [14, 27, 40, 53];

const ADDR_PRIMARY: u8 = 0x3C;
const ADDR_ALT: u8 = 0x3D;

type OledDisplay = Ssd1306<
    display_interface_i2c::I2CInterface<I2c<'static, Blocking>>,
    DisplaySize128x64,
    BufferedGraphicsMode<DisplaySize128x64>,
>;

pub struct Oled {
    disp: OledDisplay,
}

impl Oled {
    /// Probe the two standard addresses; `Some(addr)` if one ACKs (same
    /// heuristic as the retired LCD1602 probe — any I2C device at the
    /// address answers).
    pub fn probe(i2c: &mut I2c<'static, Blocking>) -> Option<u8> {
        [ADDR_PRIMARY, ADDR_ALT]
            .into_iter()
            .find(|&addr| i2c.write(addr, &[0x00]).is_ok())
    }

    /// Boot diagnostic: log every address that ACKs (I2C scan).
    pub fn scan_bus(i2c: &mut I2c<'static, Blocking>) {
        for addr in 0x08..=0x77u8 {
            if i2c.write(addr, &[0x00]).is_ok() {
                log::info!("ui: I2C device at 0x{addr:02x}");
            }
        }
    }

    pub fn new(i2c: I2c<'static, Blocking>, addr: u8) -> Self {
        let interface = I2CDisplayInterface::new_custom_address(i2c, addr);
        let disp = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        Self { disp }
    }

    pub fn init(&mut self) -> Result<(), ()> {
        self.disp.init().map_err(|_| ())
    }

    /// Render the UI lines (fixed-width ASCII) and flush.
    pub fn write_screen(&mut self, lines: &[[u8; COLS as usize]; ROWS]) -> Result<(), ()> {
        let style = MonoTextStyle::new(&FONT_8X13, BinaryColor::On);
        self.disp.clear_buffer();
        for (i, line) in lines.iter().enumerate() {
            let _ = Text::new(
                core::str::from_utf8(line).unwrap_or(""),
                Point::new(0, LINE_YS[i]),
                style,
            )
            .draw(&mut self.disp);
        }
        self.disp.flush().map_err(|_| ())
    }
}
