//! Button inputs (SPECS §9.2): active-low, internal pull-ups.
//!
//! Debounce model: a press fires on the *first* low sample (fast response —
//! the core-0 executor quantizes sampling to ~100 ms, so multi-sample press
//! confirmation would eat normal taps); the button re-arms only after two
//! consecutive high samples, which swallows contact bounce on release.
//!
//! Bench subset: RUN, MENU, ▲, ▼ — the shooting cluster buttons (BOOST,
//! FRAME, INCH) land with the real panel.

use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::peripherals::{GPIO22, GPIO25, GPIO26, GPIO5};

/// One debounced button.
struct Button {
    pin: Input<'static>,
    /// True once a press fired and the button is still in its pressed/
    /// releasing window (blocks re-trigger until two clean high samples).
    armed: bool,
    /// Consecutive high samples since release (re-arm at 2).
    high_run: u8,
}

impl Button {
    fn new(pin: Input<'static>) -> Self {
        Self {
            pin,
            armed: true,
            high_run: 2,
        }
    }

    /// Returns a press event for this sample (falls through when the
    /// button is held — no auto-repeat).
    fn sample(&mut self) -> bool {
        let pressed = !self.pin.is_high(); // active-low
        if pressed {
            self.high_run = 0;
            if self.armed {
                self.armed = false;
                return true;
            }
        } else {
            self.high_run = self.high_run.saturating_add(1);
            if self.high_run >= 2 {
                self.armed = true;
            }
        }
        false
    }

    fn held(&self) -> bool {
        !self.armed && self.high_run == 0
    }
}

pub struct Buttons {
    run: Button,
    menu: Button,
    up: Button,
    down: Button,
}

/// One sampling pass' worth of events.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Events {
    pub run_press: bool,
    pub menu_press: bool,
    pub up_press: bool,
    pub down_press: bool,
    /// Any button still held (for long-press = back detection, SPECS §9.2).
    pub up_held: bool,
    pub down_held: bool,
}

impl Buttons {
    pub fn new(
        run: GPIO5<'static>,
        menu: GPIO25<'static>,
        up: GPIO26<'static>,
        down: GPIO22<'static>,
    ) -> Self {
        let cfg = InputConfig::default().with_pull(Pull::Up);
        Self {
            run: Button::new(Input::new(run, cfg)),
            menu: Button::new(Input::new(menu, cfg)),
            up: Button::new(Input::new(up, cfg)),
            down: Button::new(Input::new(down, cfg)),
        }
    }

    /// Boot diagnostic: report each line's level (L = pressed/GND, H = idle).
    pub fn boot_report(&self) {
        log::info!(
            "ui: buttons run={} menu={} up={} down={}",
            hl(self.run.pin.is_high()),
            hl(self.menu.pin.is_high()),
            hl(self.up.pin.is_high()),
            hl(self.down.pin.is_high()),
        );
    }

    pub fn sample(&mut self) -> Events {
        Events {
            run_press: self.run.sample(),
            menu_press: self.menu.sample(),
            up_press: self.up.sample(),
            down_press: self.down.sample(),
            up_held: self.up.held(),
            down_held: self.down.held(),
        }
    }
}

fn hl(high: bool) -> &'static str {
    if high {
        "H"
    } else {
        "L"
    }
}
