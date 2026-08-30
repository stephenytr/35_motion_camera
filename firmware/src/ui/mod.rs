//! UI task (core 0): buttons, menu model (logic::menu), bench 16×2 LCD
//! (SPECS §9.1's final unit is an SSD1306 OLED — rendering is behind this
//! task either way), and UI events toward the supervisor (ARCHITECTURE §8:
//! the supervisor validates against the interlock matrix and translates to
//! director commands; the UI never enqueues commands itself).
//!
//! Bench controls: RUN, MENU, ▲, ▼. MENU enters the item view; ▲/▼
//! navigate (main), adjust (item), or cycle transport actions; long ▲/▼
//! backs out. RUN toggles run/stop from any view.

pub mod buttons;

use core::fmt::Write as _;
use core::sync::atomic::Ordering;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker};
use esp_hal::gpio::AnyPin;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::peripherals::I2C0;
use heapless::String;
use log::{info, warn};

use crate::drivers::lcd1602::{Lcd1602, COLS};
use crate::settings_store;
use crate::status::Status;
use logic::menu::{MenuItem, MenuState};

use self::buttons::Buttons;

/// What the supervisor should do about a button press.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportAction {
    Leader,
    Rewind,
    TrackBSetup,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UiEvent {
    /// RUN pressed: toggle run/stop (supervisor validates the interlock).
    RunToggle,
    /// ▲/▼ while an item is open: adjust its value.
    Adjust { item: MenuItem, up: bool },
    /// MENU while the transport submenu is open: execute the action.
    Transport(TransportAction),
}

/// UI task → supervisor (both core 0): same-core channel, CS mutex is
/// sound here (decision log #23 applies to cross-core use only).
pub static UI_EVENTS: Channel<CriticalSectionRawMutex, UiEvent, 8> = Channel::new();

const SAMPLE_MS: u64 = 30;
const TRANSPORT_ACTIONS: [TransportAction; 3] = [
    TransportAction::Leader,
    TransportAction::Rewind,
    TransportAction::TrackBSetup,
];

/// Interaction level of the menu.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Level {
    Main,
    Item,
    Transport,
}

struct UiState {
    level: Level,
    menu: MenuState,
    transport_idx: usize,
    /// Long-press counters (▲/▼ held = back, SPECS §9.2).
    hold: u8,
}

impl UiState {
    fn new() -> Self {
        Self {
            level: Level::Main,
            menu: MenuState::new(),
            transport_idx: 0,
            hold: 0,
        }
    }
}

#[embassy_executor::task]
#[allow(clippy::too_many_arguments)]
pub async fn ui_task(
    status: &'static Status,
    i2c0: I2C0<'static>,
    sda: AnyPin<'static>,
    scl: AnyPin<'static>,
    run: esp_hal::peripherals::GPIO5<'static>,
    menu: esp_hal::peripherals::GPIO25<'static>,
    up: esp_hal::peripherals::GPIO26<'static>,
    down: esp_hal::peripherals::GPIO22<'static>,
) {
    // Bench I2C pins: SDA=18, SCL=19 (decision log #29) — GPIO 21/22, the
    // chip defaults, are taken (21 = TMC CS).
    let mut i2c = I2c::new(i2c0, I2cConfig::default())
        .expect("i2c init")
        .with_sda(sda)
        .with_scl(scl);

    let addr = Lcd1602::detect_address(&mut i2c);
    let mut lcd = match addr {
        Some(addr) => {
            info!("ui: LCD 1602 detected at I2C 0x{addr:02x}");
            let mut lcd = Lcd1602::new(i2c, addr);
            match lcd.init() {
                Ok(()) => Some(lcd),
                Err(e) => {
                    warn!("ui: LCD init failed: {e:?} — running headless");
                    None
                }
            }
        }
        None => {
            // Boot diagnostic: report what actually ACKs on the bus.
            warn!("ui: no LCD at 0x27/0x3F — scanning I2C bus");
            Lcd1602::scan_bus(&mut i2c);
            None
        }
    };

    let mut buttons = Buttons::new(run, menu, up, down);
    buttons.boot_report();
    let mut ui = UiState::new();
    info!("ui: up, 16x2 LCD + RUN/MENU/▲/▼, {} ms sampling", SAMPLE_MS);

    let mut ticker = Ticker::every(Duration::from_millis(SAMPLE_MS));
    let mut last_screen: Option<([u8; COLS as usize], [u8; COLS as usize])> = None;

    loop {
        ticker.next().await;

        let ev = buttons.sample();

        // RUN is always live (shooting cluster, SPECS §9.3).
        if ev.run_press {
            let _ = UI_EVENTS.try_send(UiEvent::RunToggle);
        }

        // Long-press ▲ or ▼ = back (SPECS §9.2: "long-press = back", either
        // button alone — not a two-finger chord).
        if ev.up_held || ev.down_held {
            ui.hold += 1;
        } else {
            ui.hold = 0;
        }
        if ui.hold >= 10 {
            ui.hold = 0;
            ui.level = Level::Main;
        }

        match ui.level {
            Level::Main => {
                if ev.up_press {
                    ui.menu.next();
                }
                if ev.down_press {
                    ui.menu.prev();
                }
                if ev.menu_press {
                    ui.level = Level::Item;
                }
            }
            Level::Item => {
                let item = ui.menu.current();
                if ev.up_press {
                    let _ = UI_EVENTS.try_send(UiEvent::Adjust { item, up: true });
                }
                if ev.down_press {
                    let _ = UI_EVENTS.try_send(UiEvent::Adjust { item, up: false });
                }
                if ev.menu_press {
                    match item {
                        MenuItem::Transport => ui.level = Level::Transport,
                        _ => ui.level = Level::Main,
                    }
                }
            }
            Level::Transport => {
                if ev.up_press {
                    ui.transport_idx = (ui.transport_idx + 1) % TRANSPORT_ACTIONS.len();
                }
                if ev.down_press {
                    ui.transport_idx = (ui.transport_idx + TRANSPORT_ACTIONS.len() - 1)
                        % TRANSPORT_ACTIONS.len();
                }
                if ev.menu_press {
                    let _ = UI_EVENTS.try_send(UiEvent::Transport(TRANSPORT_ACTIONS[ui.transport_idx]));
                }
            }
        }

        // Render (only on change, to keep the I2C bus quiet).
        let screen = render(&ui, status);
        if last_screen != Some(screen) {
            last_screen = Some(screen);
            if let Some(display) = lcd.as_mut() {
                if display.write_screen(&screen.0, &screen.1).is_err() {
                    // Bus fault (e.g. display yanked): go headless.
                    warn!("ui: LCD write failed — running headless");
                    lcd = None;
                }
            }
        }
    }
}

fn fill<const N: usize>(buf: &mut [u8; N], s: &str) {
    for (i, b) in buf.iter_mut().enumerate() {
        *b = s.as_bytes().get(i).copied().unwrap_or(b' ');
    }
}

fn line1<const N: usize>(s: &str) -> [u8; N] {
    let mut buf = [b' '; N];
    fill(&mut buf, s);
    buf
}

/// Render the current view into two 16-wide ASCII lines.
fn render(ui: &UiState, status: &Status) -> ([u8; COLS as usize], [u8; COLS as usize]) {
    let settings = settings_store::settings();
    let mut l1: String<{ COLS as usize }> = String::new();
    let mut l2: String<{ COLS as usize }> = String::new();

    match ui.level {
        Level::Main => {
            // Status line (unchanged content, minus the roll count — freed
            // up to fit the state word without truncating).
            let fps = (settings.fps + 0.5) as u32;
            let state_word = if crate::rt::safe_active() {
                "SAFE"
            } else if status.door_open.load(Ordering::Relaxed) {
                "DOOR"
            } else if status.fault.load(Ordering::Relaxed) != 0 {
                "ERR"
            } else if !crate::rt::heartbeat::is_parked() {
                "RUN"
            } else {
                "IDLE"
            };
            let _ = write!(
                l1,
                "{:>3}fps F{:03} {:4}",
                fps,
                status.counter_exposed.load(Ordering::Relaxed),
                state_word
            );
            // Live cursor preview: ▲/▼ moves this, so the selected item is
            // always visible *before* MENU commits to editing it — no more
            // guessing where the cursor landed.
            let item = ui.menu.current();
            let _ = write!(l2, ">{} ", short_title(item));
            write_item_value_short(&mut l2, item, &settings, status);
        }
        Level::Item => {
            let item = ui.menu.current();
            // '*' (vs. the '>' cursor at Main) marks that ▲/▼ now edit the
            // value instead of moving the selection.
            let _ = write!(l1, "*{}", item.title());
            write_item_value(&mut l2, item, &settings, status);
        }
        Level::Transport => {
            let _ = write!(l1, "TRANSPORT");
            let action = TRANSPORT_ACTIONS[ui.transport_idx];
            let name = match action {
                TransportAction::Leader => "Track Leader",
                TransportAction::Rewind => "Rewind",
                TransportAction::TrackBSetup => "Track B Setup",
            };
            let _ = write!(l2, "{}", name);
        }
    }

    (line1(l1.as_str()), line1(l2.as_str()))
}

/// Detailed value line for the Item ("editing") view.
fn write_item_value(
    l2: &mut String<{ COLS as usize }>,
    item: MenuItem,
    settings: &logic::settings::Settings,
    status: &Status,
) {
    match item {
        MenuItem::Fps => {
            let _ = write!(l2, "{} fps", settings.fps);
        }
        MenuItem::Exposure => {
            let _ = write!(l2, "{} ms", settings.exposure_ms);
        }
        MenuItem::Roll => {
            let _ = write!(l2, "{} frames", settings.roll_frames);
        }
        MenuItem::Track => {
            let _ = write!(l2, "{:?}/{:?}", settings.mode, settings.track);
        }
        MenuItem::Boost => {
            let _ = write!(l2, "{}", if status.boost.load(Ordering::Relaxed) { "ON" } else { "OFF" });
        }
        MenuItem::Transport => {
            let _ = write!(l2, "MENU to enter");
        }
        MenuItem::Settings => {
            let _ = write!(l2, "hold {}%", settings.hold_pct);
        }
        MenuItem::About => {
            let _ = write!(l2, "35mm 1.5P v1");
        }
    }
}

/// Compact value preview for the Main ("browse") cursor line — always short
/// enough to fit next to the ">{short_title} " prefix on 16 columns, unlike
/// `write_item_value`'s detailed text.
fn write_item_value_short(
    l2: &mut String<{ COLS as usize }>,
    item: MenuItem,
    settings: &logic::settings::Settings,
    status: &Status,
) {
    match item {
        MenuItem::Fps => {
            let _ = write!(l2, "{}", settings.fps);
        }
        MenuItem::Exposure => {
            let _ = write!(l2, "{}ms", settings.exposure_ms);
        }
        MenuItem::Roll => {
            let _ = write!(l2, "{}", settings.roll_frames);
        }
        MenuItem::Track => {
            let _ = write!(l2, "{:?}", settings.track);
        }
        MenuItem::Boost => {
            let _ = write!(l2, "{}", if status.boost.load(Ordering::Relaxed) { "ON" } else { "OFF" });
        }
        MenuItem::Transport => {
            let _ = write!(l2, "->");
        }
        MenuItem::Settings => {
            let _ = write!(l2, "{}%", settings.hold_pct);
        }
        MenuItem::About => {
            let _ = write!(l2, "v1");
        }
    }
}

/// Abbreviated item name for the Main cursor line (full names live in
/// `MenuItem::title`, used at the Item/edit level where there's more room).
fn short_title(item: MenuItem) -> &'static str {
    match item {
        MenuItem::Fps => "FPS",
        MenuItem::Exposure => "EXP",
        MenuItem::Roll => "ROLL",
        MenuItem::Track => "TRACK",
        MenuItem::Boost => "BOOST",
        MenuItem::Transport => "XPORT",
        MenuItem::Settings => "SET",
        MenuItem::About => "INFO",
    }
}
