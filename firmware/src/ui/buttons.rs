//! Button inputs (SPECS §9.2): active-low, internal pull-ups, debounced by
//! double-sampling in the UI task (~20-40 ms window at 30 ms cadence).
//!
//! Bench subset: RUN, MENU, ▲, ▼ — the shooting cluster buttons (BOOST,
//! FRAME, INCH) land with the real panel.

use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::peripherals::{GPIO25, GPIO26, GPIO33, GPIO5};

/// One debounced button: a press/release event fires only after two
/// consecutive identical samples.
struct Button {
    pin: Input<'static>,
    last: bool,
    state: bool,
}

impl Button {
    fn new(pin: Input<'static>) -> Self {
        let last = !pin.is_high(); // pressed = active-low
        Self {
            pin,
            last,
            state: last,
        }
    }

    /// Returns (press, release) events for this sample.
    fn sample(&mut self) -> (bool, bool) {
        let raw = !self.pin.is_high();
        let mut press = false;
        let mut release = false;
        if raw == self.last {
            if raw != self.state {
                self.state = raw;
                press = raw;
                release = !raw;
            }
        }
        self.last = raw;
        (press, release)
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
        down: GPIO33<'static>,
    ) -> Self {
        let cfg = InputConfig::default().with_pull(Pull::Up);
        Self {
            run: Button::new(Input::new(run, cfg)),
            menu: Button::new(Input::new(menu, cfg)),
            up: Button::new(Input::new(up, cfg)),
            down: Button::new(Input::new(down, cfg)),
        }
    }

    pub fn sample(&mut self) -> Events {
        let (run_press, _) = self.run.sample();
        let (menu_press, _) = self.menu.sample();
        let (up_press, _) = self.up.sample();
        let (down_press, _) = self.down.sample();
        Events {
            run_press,
            menu_press,
            up_press,
            down_press,
            up_held: self.up.state,
            down_held: self.down.state,
        }
    }
}
