//! Pure, host-testable logic for the 35mm 1.5-perf motion picture camera.
//!
//! ARCHITECTURE.md invariant: this crate contains NO hardware access. All ISR logic,
//! profile math, interlock decisions and menu models live here and are exhaustively
//! unit-testable on a development machine (`cargo test` at the workspace root).

#![no_std]

pub mod consts;
pub mod counters;
pub mod frame_fsm;
pub mod index_watch;
pub mod interlock;
pub mod menu;
pub mod profile;
pub mod ramp;
pub mod settings;
