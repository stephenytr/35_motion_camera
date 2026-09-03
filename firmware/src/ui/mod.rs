//! UI task (core 0): buttons, menu model (logic::menu), the SSD1306 OLED
//! (SPECS §9.1), and UI events toward the supervisor (ARCHITECTURE §8: the
//! supervisor validates against the interlock matrix and translates to
//! director commands; the UI never enqueues commands itself).
//!
//! Display layout (4 lines × 16 chars, FONT_8X13):
//! ```text
//!  24fps  15.7ms     cadence + effective shutter + B (boosting)
//! F 12/228 A         exposed/roll + active track
//! P 0045  IDLE       film position + state word
//! >FPS 24            menu cursor / edit line / transport action
//! ```
//! Lines 1-3 are always the live status; line 4 is the level-dependent
//! context ('>' browsing, '*' editing).
//!
//! Bench controls: RUN, MENU, ▲, ▼ + shooting cluster (BOOST always live;
//! FRAME/INCH behind the frame-inch-buttons feature). MENU enters the item
//! view; ▲/▼ navigate (main), adjust (item), or cycle transport actions;
//! long ▲/▼ backs out. RUN toggles run/stop from any view.

pub mod buttons;
pub mod pot;

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

use crate::drivers::oled::{Oled, COLS, ROWS};
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
    /// MENU on the RESET item: zero the exposed frame counter (and persist).
    ResetCounter,
    /// Absolute fps from the pot input (whole steps, pot is the fps master
    /// while turned).
    SetFps(u8),
    /// Absolute exposure from the pot input (whole ms).
    SetExposure(u32),
    /// BOOST held/released (SPECS §9.2: hold = boost; live-ramped mid-take).
    BoostHold(bool),
    /// INCH held/released (hold = inch). S3 has internal pull-ups, so these
    /// are always live (the classic-ESP32 external-pull-up gate is gone).
    InchHold(bool),
    /// FRAME pressed: run exactly one frame.
    Frame,
}

/// UI task → supervisor (both core 0): same-core channel, CS mutex is
/// sound here (decision log #23 applies to cross-core use only).
pub static UI_EVENTS: Channel<CriticalSectionRawMutex, UiEvent, 8> = Channel::new();

#[cfg(feature = "debug-prints")]
static UI_TICK: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

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
    menu: esp_hal::peripherals::GPIO26<'static>,
    up: esp_hal::peripherals::GPIO29<'static>,
    down: esp_hal::peripherals::GPIO28<'static>,
    boost: esp_hal::peripherals::GPIO48<'static>,
    frame: esp_hal::peripherals::GPIO12<'static>,
    inch: esp_hal::peripherals::GPIO39<'static>,
) {
    // I2C pins: SDA=18, SCL=19 (decision log #29) — the chip-default 21/22
    // pair is taken (21 = takeup DIR, 22 = ▼).
    //
    // `ui_task` shares the core-0 executor with supervisor/power/storage/
    // wdt — a panic here has the same blast radius as a supervisor panic
    // (it stalls every core-0 task, including `wdt_task`, which then stops
    // feeding the RTC watchdog). I2C peripheral construction failing is
    // unlikely, but there is no reason to risk the whole command plane on
    // it when the OLED is already treated as optional everywhere else
    // (probe failure, init failure, and bus faults during operation all
    // degrade to headless mode) — do the same here.
    // Bounded I2C: esp-hal's default config has NO software timeout — a
    // wedged bus (SDA held low by a glitching display, motor noise) hangs
    // the blocking write forever, and since every core-0 task shares one
    // cooperative executor, that single hang starved wdt_task and the RTC
    // watchdog rebooted the chip. 400 kHz shrinks each flush (~25 ms vs
    // ~120 ms at 100 kHz), and a 300 ms per-transaction deadline turns a
    // stuck bus into an Err that degrades to headless instead of a freeze.
    let i2c_cfg = I2cConfig::default()
        .with_frequency(esp_hal::time::Rate::from_khz(400))
        .with_software_timeout(esp_hal::i2c::master::SoftwareTimeout::Transaction(
            esp_hal::time::Duration::from_millis(300),
        ));
    let mut display = match I2c::new(i2c0, i2c_cfg) {
        Ok(i2c) => {
            let mut i2c = i2c.with_sda(sda).with_scl(scl);
            match Oled::probe(&mut i2c) {
                Some(addr) => {
                    info!("ui: SSD1306 OLED detected at I2C 0x{addr:02x}");
                    let mut display = Oled::new(i2c, addr);
                    match display.init() {
                        Ok(()) => Some(display),
                        Err(_) => {
                            warn!("ui: OLED init failed — running headless");
                            None
                        }
                    }
                }
                None => {
                    // Boot diagnostic: report what actually ACKs on the bus.
                    warn!("ui: no OLED at 0x3C/0x3D — scanning I2C bus");
                    Oled::scan_bus(&mut i2c);
                    None
                }
            }
        }
        Err(e) => {
            warn!("ui: I2C init failed ({e:?}) — running headless");
            None
        }
    };

    let mut buttons = Buttons::new(run, menu, up, down, boost, frame, inch);
    buttons.boot_report();
    let mut ui = UiState::new();
    info!(
        "ui: up, 128x64 OLED + RUN/MENU/▲/▼/BOOST/FRAME/INCH, {} ms sampling",
        SAMPLE_MS
    );

    let mut ticker = Ticker::every(Duration::from_millis(SAMPLE_MS));
    let mut last_screen: Option<Screen> = None;
    let mut last_flush = embassy_time::Instant::now();
    let mut boost_prev = false;
    let mut inch_prev = false;

    loop {
        ticker.next().await;
        crate::liveness::LIVENESS.bump_ui();

        let ev = buttons.sample();

        // Shooting cluster is always live (SPECS §9.3), like RUN.
        if ev.run_press {
            let _ = UI_EVENTS.try_send(UiEvent::RunToggle);
        }
        {
            if ev.frame_press {
                let _ = UI_EVENTS.try_send(UiEvent::Frame);
            }
            if ev.inch_press {
                let _ = UI_EVENTS.try_send(UiEvent::InchHold(true));
            } else if inch_prev && !ev.inch_held {
                let _ = UI_EVENTS.try_send(UiEvent::InchHold(false));
            }
            inch_prev = ev.inch_held;
        }
        if ev.boost_press {
            let _ = UI_EVENTS.try_send(UiEvent::BoostHold(true));
        } else if boost_prev && !ev.boost_held {
            let _ = UI_EVENTS.try_send(UiEvent::BoostHold(false));
        }
        boost_prev = ev.boost_held;

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
                        MenuItem::Reset => {
                            let _ = UI_EVENTS.try_send(UiEvent::ResetCounter);
                            ui.level = Level::Main;
                        }
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

        // Render only on change, throttled to ~20 Hz: at 30+ fps the frame
        // counter changes every ~30 ms, which drove one ~25-40 ms blocking
        // I2C flush per frame — the bus was in use nearly 100% of the time
        // and motor noise at those step rates wedged it. 50 ms between
        // flushes keeps the OLED responsive but the bus mostly idle; the
        // screen always shows the latest state after the cap.
        const MIN_FLUSH_INTERVAL: Duration = Duration::from_millis(50);
        let screen = render(&ui, status);
        if last_screen != Some(screen) && last_flush.elapsed() >= MIN_FLUSH_INTERVAL {
            last_screen = Some(screen);
            last_flush = embassy_time::Instant::now();
            if let Some(disp) = display.as_mut() {
                #[cfg(feature = "debug-prints")]
                {
                    let n = UI_TICK.fetch_add(1, Ordering::Relaxed);
                    if n % 33 == 0 {
                        log::info!("dbg: ui alive");
                    }
                    log::info!("dbg: ui draw begin");
                }
                if disp.write_screen(&screen).is_err() {
                    // Bus fault (e.g. display yanked): go headless.
                    warn!("ui: OLED write failed — running headless");
                    display = None;
                }
                #[cfg(feature = "debug-prints")]
                log::info!("dbg: ui drawn");
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

/// One rendered frame: 4 fixed-width ASCII lines (see the module docs for
/// the layout).
pub type Screen = [[u8; COLS as usize]; ROWS];

/// Render the current view into four 16-wide ASCII lines. Lines 1-3 are the
/// persistent status (cadence/shutter, counters, position/state); line 4 is
/// the menu context (cursor preview while browsing, the value being edited,
/// or the selected transport action).
fn render(ui: &UiState, status: &Status) -> Screen {
    let settings = settings_store::settings();

    // L1: cadence + effective shutter time + boost flag. The exposure shown
    // is what the shutter will actually do at the current cadence (the
    // requested value is clamped to the frame period — showing the raw
    // setting was a display-vs-reality lie, audit finding #5).
    let fps = (settings.fps + 0.5) as u32;
    let eff_ms = logic::frame_fsm::params_for(settings.fps, settings.exposure_ms, true)
        .exp_us as f32
        / 1000.0;
    let boost = if status.boost.load(Ordering::Relaxed) { "B" } else { " " };
    let mut l1: String<{ COLS as usize }> = String::new();
    let _ = write!(l1, "{fps:>3}fps {eff_ms:>5.1}ms {boost}");

    // L2: exposure counter / roll length + active track.
    let track = match settings.track {
        logic::settings::Track::A => "A",
        logic::settings::Track::B => "B",
    };
    let mut l2: String<{ COLS as usize }> = String::new();
    let _ = write!(
        l2,
        "F{:>3}/{} {}",
        status.counter_exposed.load(Ordering::Relaxed),
        settings.roll_frames,
        track
    );

    // L3: film position from the datum + state word.
    let state_word = if crate::rt::safe_active() {
        "SAFE"
    } else if status.door_open.load(Ordering::Relaxed) {
        "DOOR"
    } else if status.fault.load(Ordering::Relaxed) != 0 {
        "ERR"
    } else if !crate::rt::heartbeat::is_parked() {
        "RUN"
    } else if status.batt_warn.load(Ordering::Relaxed) {
        // SPECS §11/§6.2: 19.8 V warn-only — displayed, doesn't stop.
        "BATLOW"
    } else {
        "IDLE"
    };
    let mut l3: String<{ COLS as usize }> = String::new();
    let _ = write!(l3, "P{:>5} {:>4}", crate::rt::position::frames(), state_word);

    // L4: menu context.
    let mut l4: String<{ COLS as usize }> = String::new();
    match ui.level {
        Level::Main => {
            // Live cursor preview: ▲/▼ moves this, so the selected item is
            // always visible *before* MENU commits to editing it.
            let item = ui.menu.current();
            let _ = write!(l4, ">{} ", short_title(item));
            write_item_value_short(&mut l4, item, &settings, status, eff_ms);
        }
        Level::Item => {
            // '*' (vs. the '>' cursor at Main) marks that ▲/▼ now edit the
            // value instead of moving the selection.
            let item = ui.menu.current();
            let _ = write!(l4, "*{} ", short_title(item));
            write_item_value_short(&mut l4, item, &settings, status, eff_ms);
        }
        Level::Transport => {
            let action = TRANSPORT_ACTIONS[ui.transport_idx];
            let name = match action {
                TransportAction::Leader => "LEAD",
                TransportAction::Rewind => "RWD",
                TransportAction::TrackBSetup => "T-B",
            };
            let _ = write!(l4, ">XPORT {} ", name);
        }
    }

    [
        line1(l1.as_str()),
        line1(l2.as_str()),
        line1(l3.as_str()),
        line1(l4.as_str()),
    ]
}

/// Compact value for the context line — always short enough to fit next to
/// the ">ITEM "/"*ITEM " prefix on 16 columns. `eff_ms` is the effective
/// (clamped) shutter time at the current cadence: EXPOSURE shows
/// "requested>effective" so a pot value the frame period can't honor is
/// visibly clamped rather than silently ignored.
fn write_item_value_short(
    l: &mut String<{ COLS as usize }>,
    item: MenuItem,
    settings: &logic::settings::Settings,
    status: &Status,
    eff_ms: f32,
) {
    match item {
        MenuItem::Fps => {
            let _ = write!(l, "{}", settings.fps);
        }
        MenuItem::Exposure => {
            let _ = write!(l, "{}>{}", settings.exposure_ms, EffDisplay(eff_ms));
        }
        MenuItem::Roll => {
            let _ = write!(l, "{}", settings.roll_frames);
        }
        MenuItem::Track => {
            let _ = write!(l, "{:?}", settings.track);
        }
        MenuItem::Boost => {
            let _ = write!(l, "{}", if status.boost.load(Ordering::Relaxed) { "ON" } else { "OFF" });
        }
        MenuItem::Transport => {
            let _ = write!(l, "MENU");
        }
        MenuItem::Reset => {
            let _ = write!(l, "MENU");
        }
        MenuItem::Settings => {
            let _ = write!(l, "{}%", settings.hold_pct);
        }
        MenuItem::About => {
            let _ = write!(l, "v1");
        }
    }
}

/// Formats the effective shutter time without a trailing ".0" (16 columns
/// are tight, and heapless `write!` can't pick precision per magnitude).
struct EffDisplay(f32);

impl core::fmt::Display for EffDisplay {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if self.0 >= 100.0 {
            write!(f, "{:.0}ms", self.0)
        } else {
            write!(f, "{:.1}ms", self.0)
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
        MenuItem::Reset => "RESET",
        MenuItem::Settings => "SET",
        MenuItem::About => "INFO",
    }
}
