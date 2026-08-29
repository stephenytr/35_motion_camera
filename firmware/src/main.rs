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
mod power;
mod rt;
mod status;
mod storage;
mod supervisor;
mod ui;

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

    // Command queue: one-time split into the two single-owner halves.
    let cmd_q: &'static mut command::CmdQueue =
        command::CMD_QUEUE_CELL.init(command::CmdQueue::new());
    let (cmd_tx, cmd_rx) = cmd_q.split();

    // Timers (decision log #15): the esp_rtos scheduler owns timg1.1 (P1,
    // core 0); timg0.0/0.1 and timg1.0 belong to the RT plane on core 1.
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    esp_rtos::start(timg1.timer1, peripherals.FROM_CPU_INTR0);

    // Bench timing strobe: user LED on GPIO 13 (consts::DEBUG_STROBE_GPIO).
    let strobe = esp_hal::gpio::Output::new(
        peripherals.GPIO13,
        esp_hal::gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );

    // Command plane tasks on core 0 (ARCHITECTURE §5.1).
    spawner
        .spawn(supervisor::supervisor_task(&fault::EVENTS, cmd_tx, &status::STATUS).unwrap());
    spawner.spawn(ui::ui_task(&status::STATUS).unwrap());
    spawner.spawn(power::power_task(&fault::EVENTS, &status::STATUS).unwrap());
    spawner.spawn(storage::storage_task(&status::STATUS).unwrap());

    // Core 1: the director has its own executor (ARCHITECTURE §5.1, §6).
    static APP_CORE_STACK: StaticCell<Stack<16384>> = StaticCell::new();
    let app_core_stack = APP_CORE_STACK.init(Stack::new());

    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        peripherals.FROM_CPU_INTR1,
        app_core_stack,
        move || {
            // RT plane lives entirely on core 1 (ARCHITECTURE §2): bind the
            // heartbeat ISR here so the handler runs on this core.
            rt::heartbeat::init(timg0.timer0, strobe);

            static EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();
            let executor = EXECUTOR.init(esp_rtos::embassy::Executor::new());
            executor.run(|spawner| {
                spawner.spawn(
                    director::director_task(&fault::EVENTS, cmd_rx, &status::STATUS)
                        .unwrap(),
                );
            });
        },
    );

    // Keep the boot task alive as a slow heartbeat while the executors run.
    let mut ticker = Ticker::every(Duration::from_secs(5));
    loop {
        ticker.next().await;
        log::info!("main: both executors alive");
    }
}
