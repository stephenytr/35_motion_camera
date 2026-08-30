//! HD44780 16×2 character LCD over a PCF8574 I2C backpack — the bench
//! display (SPECS §9.1's final unit is an SSD1306 OLED; this driver is
//! presentation-agnostic: the UI task renders lines, the driver writes
//! them).
//!
//! Standard backpack pin map: RS=0x01, RW=0x02, E=0x04, backlight=0x08,
//! D4=0x10 .. D7=0x80. 4-bit mode with fixed timing delays (RW is tied
//! write-only on most backpacks, so no busy-flag read).
//!
//! Address: 0x27 on most modules, 0x3F on some — auto-detected at boot by
//! `detect_address` (first NACK-free address wins).

use esp_hal::delay::Delay;
use esp_hal::i2c::master::{Error as I2cError, I2c};
use esp_hal::Blocking;

pub const ADDR_A: u8 = 0x27;
pub const ADDR_B: u8 = 0x3F;

const BL: u8 = 0x08;
const RS: u8 = 0x01;
const EN: u8 = 0x04;

/// Columns per line (physical display width).
pub const COLS: u8 = 16;

pub struct Lcd1602 {
    i2c: I2c<'static, Blocking>,
    addr: u8,
}

impl Lcd1602 {
    pub fn new(i2c: I2c<'static, Blocking>, addr: u8) -> Self {
        Self { i2c, addr }
    }

    /// Probe both common backpack addresses; `None` if neither ACKs
    /// (no display wired — the UI task then skips rendering).
    pub fn detect_address(i2c: &mut I2c<'static, Blocking>) -> Option<u8> {
        for addr in [ADDR_A, ADDR_B] {
            // A zero-length-ish probe: write a single harmless byte. The
            // HD44780 ignores it while idle; a NACK means no device.
            if i2c.write(addr, &[0x00]).is_ok() {
                return Some(addr);
            }
        }
        None
    }

    fn write_raw(&mut self, byte: u8) -> Result<(), I2cError> {
        self.i2c.write(self.addr, &[byte])
    }

    /// One 4-bit nibble transfer with enable strobe.
    fn nibble(&mut self, rs: bool, nib: u8) -> Result<(), I2cError> {
        let base = BL | if rs { RS } else { 0 } | (nib & 0x0F) << 4;
        self.write_raw(base | EN)?;
        self.write_raw(base)?;
        Ok(())
    }

    fn cmd(&mut self, c: u8) -> Result<(), I2cError> {
        self.nibble(false, c >> 4)?;
        self.nibble(false, c & 0x0F)
    }

    fn data(&mut self, c: u8) -> Result<(), I2cError> {
        self.nibble(true, c >> 4)?;
        self.nibble(true, c & 0x0F)
    }

    /// 4-bit-mode bring-up handshake (HD44780 datasheet figure 24).
    /// Caller must have waited ≥ 40 ms since power-on.
    pub fn init(&mut self) -> Result<(), I2cError> {
        let delay = Delay::new();

        self.nibble(false, 0x3)?;
        delay.delay_millis(5);
        self.nibble(false, 0x3)?;
        delay.delay_micros(150);
        self.nibble(false, 0x3)?;
        delay.delay_micros(150);
        self.nibble(false, 0x2)?; // switch to 4-bit mode
        delay.delay_micros(150);

        self.cmd(0x28)?; // function set: 4-bit, 2 lines, 5x8
        delay.delay_micros(60);
        self.cmd(0x0C)?; // display on, cursor off, no blink
        delay.delay_micros(60);
        self.clear()?;
        self.cmd(0x06)?; // entry mode: left-to-right, no shift
        delay.delay_micros(60);
        Ok(())
    }

    pub fn clear(&mut self) -> Result<(), I2cError> {
        self.cmd(0x01)?;
        Delay::new().delay_millis(2);
        Ok(())
    }

    /// Move the cursor to (line, col), 0-based.
    pub fn position(&mut self, line: u8, col: u8) -> Result<(), I2cError> {
        let base = match line {
            0 => 0x00,
            _ => 0x40,
        };
        self.cmd(0x80 | (base + col))
    }

    /// Write a line, padded/truncated to the display width.
    pub fn write_line(&mut self, line: u8, text: &str) -> Result<(), I2cError> {
        self.position(line, 0)?;
        let mut n = 0u8;
        for b in text.bytes().take(COLS as usize) {
            self.data(b)?;
            n += 1;
        }
        while n < COLS {
            self.data(b' ')?;
            n += 1;
        }
        Ok(())
    }

    /// Write both lines from fixed-width ASCII buffers.
    pub fn write_screen(&mut self, l1: &[u8; COLS as usize], l2: &[u8; COLS as usize]) -> Result<(), I2cError> {
        self.write_line(0, core::str::from_utf8(l1).unwrap_or(""))?;
        self.write_line(1, core::str::from_utf8(l2).unwrap_or(""))
    }
}
