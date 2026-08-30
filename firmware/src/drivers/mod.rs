//! Driver layer (ARCHITECTURE §7). Bring-up order:
//!
//! TODO: `takeup` — LEDC + DIR, feedforward duty table vs fps
//! TODO: `oled` — async I2C ssd1306 + embedded-graphics, core 0
//! TODO: `buttons` — async GPIO with task-side debounce
//! TODO: `storage` — embedded-storage flash, ping-pong 4 KB, CRC32 records

pub mod lcd1602;
pub mod rmt_step;
pub mod shutter;
pub mod tmc;
