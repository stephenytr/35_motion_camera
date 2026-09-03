//! Button inputs (SPECS §9.2): active-low with internal pull-ups — the
//! ESP32-S3 has pull-ups on every GPIO, so no external resistors are needed.
//!
//! Debounce model: a press fires on the *first* low sample (fast response —
//! the core-0 executor quantizes sampling to ~100 ms, so multi-sample press
//! confirmation would eat normal taps); the button re-arms only after two
//! consecutive high samples, which swallows contact bounce on release.
//!
//! Bench set: RUN, MENU, ▲, ▼ + the shooting cluster (BOOST hold, FRAME,
//! INCH hold — SPECS §9.3). DOOR is the real door-switch input (GPIO4,
//! rt::door), bench-wired as a momentary button: hold = closed.
//!
//! S3 pin notes (decision #33): GPIO22-25 don't exist on the ESP32-S3, and
//! GPIO33-37 are consumed by octal PSRAM on the N16R8 module — MENU/▲/▼
//! moved to 26/29/28, BOOST to 48, FRAME to 12.

use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::peripherals::{GPIO5, GPIO12, GPIO26, GPIO28, GPIO29, GPIO39, GPIO48};

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
    boost: Button,
    frame: Button,
    inch: Button,
}

/// One sampling pass' worth of events.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Events {
    pub run_press: bool,
    pub menu_press: bool,
    pub up_press: bool,
    pub down_press: bool,
    pub boost_press: bool,
    pub frame_press: bool,
    pub inch_press: bool,
    /// Still-held states (up/down: long-press = back, SPECS §9.2; boost/
    /// inch: hold-to-activate, SPECS §9.2).
    pub up_held: bool,
    pub down_held: bool,
    pub boost_held: bool,
    pub inch_held: bool,
}

impl Buttons {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        run: GPIO5<'static>,
        menu: GPIO26<'static>,
        up: GPIO29<'static>,
        down: GPIO28<'static>,
        boost: GPIO48<'static>,
        frame: GPIO12<'static>,
        inch: GPIO39<'static>,
    ) -> Self {
        let cfg = InputConfig::default().with_pull(Pull::Up);
        Self {
            run: Button::new(Input::new(run, cfg)),
            menu: Button::new(Input::new(menu, cfg)),
            up: Button::new(Input::new(up, cfg)),
            down: Button::new(Input::new(down, cfg)),
            boost: Button::new(Input::new(boost, cfg)),
            frame: Button::new(Input::new(frame, cfg)),
            inch: Button::new(Input::new(inch, cfg)),
        }
    }

    /// Boot diagnostic: report each line's level (L = pressed/GND, H = idle).
    pub fn boot_report(&self) {
        log::info!(
            "ui: buttons run={} menu={} up={} down={} boost={} frame={} inch={}",
            hl(self.run.pin.is_high()),
            hl(self.menu.pin.is_high()),
            hl(self.up.pin.is_high()),
            hl(self.down.pin.is_high()),
            hl(self.boost.pin.is_high()),
            hl(self.frame.pin.is_high()),
            hl(self.inch.pin.is_high()),
        );
    }

    pub fn sample(&mut self) -> Events {
        Events {
            run_press: self.run.sample(),
            menu_press: self.menu.sample(),
            up_press: self.up.sample(),
            down_press: self.down.sample(),
            boost_press: self.boost.sample(),
            frame_press: self.frame.sample(),
            inch_press: self.inch.sample(),
            up_held: self.up.held(),
            down_held: self.down.held(),
            boost_held: self.boost.held(),
            inch_held: self.inch.held(),
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
