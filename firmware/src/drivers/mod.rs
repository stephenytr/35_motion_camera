//! Driver layer (ARCHITECTURE §7). Core-1 drivers (motion + shutter
//! waveform) are constructed in the core-1 closure; the OLED (core 0)
//! is owned by the UI task.

pub mod oled;
pub mod rmt_step;
pub mod shutter;
pub mod takeup;
pub mod tmc2209;
