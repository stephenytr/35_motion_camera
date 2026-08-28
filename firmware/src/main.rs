//! Firmware bring-up skeleton — structure per ARCHITECTURE.md §11.
//!
//! Boot order (ARCHITECTURE §11): safe state FIRST, then esp-hal init, then the
//! two-plane bootstrap. The placeholder loop is replaced at bring-up by:
//! core 0 embassy executor (ui / supervisor / power / storage) and core 1
//! (director task + raw ISRs at P2/P3).

#![no_std]
#![no_main]

mod consts;
mod director;
mod drivers;
mod fault;
mod rt;
mod status;
mod storage;
mod supervisor;
mod ui;

#[esp_hal::entry]
fn main() -> ! {
    rt::safe_state();

    let _peripherals = esp_hal::init(esp_hal::Config::default());

    loop {}
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    rt::safe_state();
    loop {}
}
