//! Driver layer (ARCHITECTURE §7). Bring-up order:
//!
//! TODO: `tmc5160` — blocking SPI device wrapper, core 1 (director-owned)
//! TODO: `shutter` — LEDC duty wrapper (pull/hold/off)
//! TODO: `takeup` — LEDC + DIR, feedforward duty table vs fps
//! TODO: `oled` — async I2C ssd1306 + embedded-graphics, core 0
//! TODO: `buttons` — async GPIO with task-side debounce
//! TODO: `storage` — embedded-storage flash, ping-pong 4 KB, CRC32 records
