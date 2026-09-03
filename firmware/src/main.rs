//! Firmware bring-up — M1: dual-core embassy executors (ARCHITECTURE §5.1).
//!
//! Core 0: ui / power / storage / supervisor tasks on the esp-rtos thread-mode
//! executor. Core 1: the director task alone (it owns SPI2 in M3; ISRs will
//! preempt it at P2/P3 in M2).
//!
//! Inter-plane links (ARCHITECTURE §5.2):
//!   - commands: heapless spsc queue, supervisor -> director (lock-free)
//!   - events:   embassy channel, director/ISRs -> tasks (try_send only)
//!   - status:   field-atomic static read lock-free by the UI

#![no_std]
#![no_main]

mod command;
mod consts;
mod director;
mod drivers;
mod fault;
mod liveness;
mod power;
mod rt;
mod settings_store;
mod status;
mod storage;
mod supervisor;
mod ui;
mod wdt;

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_hal::system::Stack;
use esp_hal::timer::timg::TimerGroup;
use log::LevelFilter;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger(LevelFilter::Info);
    rt::safe_state();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Fault marker from the last run (ARCHITECTURE §4.4): the deadman writes
    // it, the RTC watchdog reboots, and we report it here before clearing.
    // Surfaced into `status.fault` directly (not just logged) so the OLED
    // shows "ERR" instead of a clean-looking "IDLE" after a real fault
    // caused the reboot — SPECS §11 "errors latch until user acknowledgement"
    // otherwise silently didn't apply to the one class of fault severe
    // enough to reboot the whole chip. `RunToggle` (supervisor.rs) treats a
    // RUN press while a fault is latched as an acknowledgement.
    if let Some(code) = fault::marker_take() {
        log::error!("boot: fault marker {code:#010x} — ERROR WD");
        status::STATUS
            .fault
            .store(code, core::sync::atomic::Ordering::Relaxed);
    }

    // Brownout auto-recover (SPECS §11, ARCHITECTURE §6): a VDD dip resets
    // the whole chip (no time to run the deadman marker), so the only
    // evidence is the RTC_CNTL reset-reason register. If that's what put us
    // here, the supervisor auto-issues Recover once it's up instead of
    // requiring a manual command — the same creep-to-next-index-edge job
    // used for a manual recovery.
    let reason = esp_hal::rtc_cntl::reset_reason(esp_hal::system::Cpu::ProCpu);
    if let Some(r) = reason {
        // Log every reset class, not just brownout: a hard-freeze crash
        // (unfed RWDT) shows up as SysRtcWdt here even though the deadman
        // marker was never written, which is the only trace we get.
        log::warn!("boot: reset reason = {r:?}");
    }
    let brownout = reason == Some(esp_hal::rtc_cntl::SocResetReason::SysBrownOut);
    if brownout {
        log::warn!("boot: brownout reset detected — auto-recover pending");
    }

    // Command queue: one-time split into the two single-owner halves.
    let cmd_q: &'static mut command::CmdQueue =
        command::CMD_QUEUE_CELL.init(command::CmdQueue::new());
    let (cmd_tx, cmd_rx) = cmd_q.split();

    // Timers (decision log #15): the esp_rtos scheduler owns timg1.1 (P1,
    // core 0); timg0.0/0.1 and timg1.0 belong to the RT plane on core 1.
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    esp_rtos::start(timg1.timer1, peripherals.FROM_CPU_INTR0);

    // Command plane tasks on core 0 (ARCHITECTURE §5.1).
    spawner.spawn(
        supervisor::supervisor_task(&fault::EVENTS, cmd_tx, &status::STATUS, brownout).unwrap(),
    );
    spawner.spawn(
        ui::ui_task(
            &status::STATUS,
            peripherals.I2C0,
            esp_hal::gpio::AnyPin::from(peripherals.GPIO18), // LCD SDA
            esp_hal::gpio::AnyPin::from(peripherals.GPIO17), // LCD SCL (19 = USB D- on S3)
            peripherals.GPIO5,  // RUN
            peripherals.GPIO26, // MENU (22-25 don't exist on the S3)
            peripherals.GPIO29, // ▲
            peripherals.GPIO28, // ▼
            peripherals.GPIO48, // BOOST (33-37 = octal PSRAM on N16R8)
            peripherals.GPIO12, // FRAME
            peripherals.GPIO39, // INCH
        )
        .unwrap(),
    );
    spawner.spawn(power::power_task(&status::STATUS).unwrap());
    spawner.spawn(storage::storage_task(&status::STATUS).unwrap());
    spawner.spawn(wdt::wdt_task(peripherals.RTC_TIMER).unwrap());
    spawner.spawn(
        ui::pot::pot_task(
            peripherals.ADC1,
            peripherals.GPIO1, // fps pot (ADC1_CH0)
            peripherals.GPIO2, // exposure pot (ADC1_CH1)
        )
        .unwrap(),
    );

    // Core 1: the director has its own executor (ARCHITECTURE §5.1, §6).
    static APP_CORE_STACK: StaticCell<Stack<16384>> = StaticCell::new();
    let app_core_stack = APP_CORE_STACK.init(Stack::new());

    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        peripherals.FROM_CPU_INTR1,
        app_core_stack,
        move || {
            // RT plane lives entirely on core 1 (ARCHITECTURE §2): bind the
            // ISRs here so their handlers run on this core, and construct the
            // non-Send HAL drivers here too.
            rt::deadman::init();
            rt::heartbeat::init(timg0.timer0);
            rt::shutter::init(timg1.timer0, timg0.timer1);
            rt::door::init(peripherals.IO_MUX, peripherals.GPIO4);
            let ledc = esp_hal::ledc::Ledc::new(peripherals.LEDC);
            drivers::shutter::init(&ledc, peripherals.GPIO27); // MOSFET gate
            drivers::rmt_step::init(peripherals.RMT, peripherals.GPIO15);
            drivers::takeup::init_timer_and_channel(peripherals.GPIO38); // takeup STEP

            // Two TMC2209 axes in pin mode (decision log #32): transport
            // keeps the old STEP/DIR/EN pins; takeup reuses the retired SPI
            // pins for EN/DIR/STEP (STEP = LEDC pulse train).
            let tmc = drivers::tmc2209::Tmc2209::new(
                peripherals.GPIO14, // transport EN
                peripherals.GPIO32, // transport DIR
            );
            let takeup = drivers::takeup::Takeup::new(
                peripherals.GPIO13, // takeup EN (was GPIO12 — boot strapping pin conflict)
                peripherals.GPIO21, // takeup DIR
            );

            static EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();
            let executor = EXECUTOR.init(esp_rtos::embassy::Executor::new());
            executor.run(|spawner| {
                spawner.spawn(
                    director::director_task(&fault::EVENTS, cmd_rx, &status::STATUS, tmc, takeup)
                        .unwrap(),
                );
            });
        },
    );

    // Keep the boot task alive as a slow heartbeat while the executors run.
    #[cfg(feature = "debug-prints")]
    let hal_t0 = esp_hal::time::Instant::now();
    let mut ticker = Ticker::every(Duration::from_secs(5));
    loop {
        ticker.next().await;
        log::info!("main: both executors alive");
        #[cfg(feature = "debug-prints")]
        log::info!(
            "dbg: hal clock elapsed = {} ms",
            (esp_hal::time::Instant::now() - hal_t0).as_millis()
        );
    }
}

/// Panic: drive every actuator safe, then spin (ARCHITECTURE §4.6). Nothing
/// feeds the RTC watchdog from inside this loop — `wdt_task` runs on the
/// same core-0 executor as every other command-plane task, so a panic here
/// stalls it along with everything else; the *unfed* RTC watchdog is what
/// eventually reboots the system, landing on the fault marker (if any) at
/// the next boot. Safe state is already applied before we start spinning,
/// so the reboot delay is inert from a safety standpoint.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    rt::safe_state();
    esp_println::println!("PANIC: {}", info);
    loop {
        core::hint::spin_loop();
    }
}

