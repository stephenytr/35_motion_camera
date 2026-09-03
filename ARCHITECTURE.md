# Firmware Architecture — 35mm 1.5-Perf Camera

**Document**: ARCHITECTURE.md · v1.0 · 2026-08-28
**Inputs**: SPECS.md v1.0 (all [LOCKED] items are binding here)
**Target**: ESP32-S3 (Xtensa LX7 dual core) · esp-hal 1.x · no_std + embassy · no heap

**Legend**: **[LOCKED]** from SPECS · **[ARCH]** decision of this document · **[VERIFY]** pin down at bring-up.

---

## 1. The Central Problem

This firmware mixes two kinds of work:

- **Hard real time**: the per-frame cycle (pulldown 14–17 ms → settle 3 ms → exposure)
  must execute with µs-level, *guaranteed* phase relationships — shutter must never open
  on moving film, cadence must never jitter visibly.
- **Soft real time**: menus, OLED redraw, settings persistence, ADC supervision —
  tens-of-ms latencies are fine.

Async tasks (embassy) give comfortable soft timing but **cannot guarantee** wake-up
latency: executor queuing, flash-write stalls, and ISR preemption all add jitter. Flash
erase in particular can stall cores for milliseconds.

**Therefore: two planes.** All frame-cycle timing lives in hardware timers and raw ISRs
("the RT plane"). Everything else is embassy tasks ("the command plane"). Tasks *command*
the RT plane; ISRs *execute* the frame. No task is ever on the critical timing path.

```
            COMMAND PLANE (soft)                    RT PLANE (hard)
┌──────────────────────────────────────┐   ┌────────────────────────────────────┐
│  CORE 0 · embassy executor @P1       │   │  CORE 1 · raw ISRs @P2/P3          │
│                                      │   │                                    │
│  supervisor ── commands ─────────────┼──►│  director task (embassy @P1, the   │
│  ui (buttons, menus, OLED)           │   │  only task on core 1): jobs,       │
│  storage (flash, idle-only writes)   │   │  ramps, profile rebuild, TMC SPI   │
│  power (ADC), log (USB-SJ)           │   │                                    │
│                                      │   │  ISRs (all #[ram]):                │
│  async drivers: I2C, buttons GPIO,   │   │   P3 deadman timer  (last-line)    │
│  ADC — all constructed on core 0     │   │   P3 door interlock (safe state)   │
│                                      │   │   P2 heartbeat  (frame phase FSM)  │
│                                      │   │   P2 exposure / peak-hold timers  │
│                                      │   │   P2 index GPIO    (watchdog)     │
│                                      │   │                                    │
│                                      │   │  Peripherals: timg0.0/.1, timg1.0/│
│                                      │   │  .1, RMT TX (DMA), LEDC ×2,       │
│                                      │   │  GPIO outputs, SPI2 → TMC5160     │
└──────────────────────────────────────┘   └────────────────────────────────────┘
```

### The Constitution (five invariants — every review checks these)

1. **Frame timing never depends on task scheduling.** Only hardware timers, RMT, LEDC,
   and raw ISRs generate or sequence timing.
2. **Safety actions happen in ISRs, not tasks.** Door-open → safe state is register
   writes in the GPIO ISR itself. Tasks only *report*.
3. **ISRs are pure-and-tiny**: no float (Xtensa: FPU forbidden in handlers), no
   allocation, no logging, no blocking. ISR logic is extracted into pure state machines
   in the `logic` crate, returning action records that thin ISR wrappers apply.
4. **No heap.** `heapless` + statics. `esp-alloc` may be introduced later only with
   sign-off.
5. **Flash writes only when transport is idle** (IDLE / DOOR / parked-ERROR). Erases
   stall cores; cadence determinism wins. See §12 for the SPECS deviation this causes.

---

## 2. Core & Interrupt Plan [ARCH]

esp-hal on Xtensa exposes **Priority1..Priority3** (verified from docs). Async drivers
install their own handlers at construction; we leave all async drivers at default P1
(defaults are the supported configuration) and reserve P2/P3 for raw handlers.

| Owner | Runs on | Priority | Role |
|---|---|---|---|
| Deadman timer ISR (systimer unit-1 alarm) | core 1 | **P3** | last-line safe state, nothing preempts it but itself |
| Door interlock ISR (GPIO) | core 1 | **P3** | immediate safe state on door open |
| Heartbeat ISR (timg0.0) | core 1 | **P2** | frame-phase FSM advance |
| Exposure-end / peak-hold ISRs (timg0.1, timg1.0) | core 1 | **P2** | shutter timing |
| Index GPIO ISR | core 1 | **P2** | step-loss watchdog feed |
| Embassy executors (both) | core 0 + core 1 | P1 | all async drivers at P1 |

Rules:

- **All raw handlers bound to core 1** (ISRs co-reside, priority order is local, no
  cross-core ISR jitter).
- **SPI2 (TMC5160) constructed on core 1** and owned by the director task — esp-hal
  binds async/blocking driver internals to the initializing core. OLED I²C, button GPIOs,
  ADC constructed on core 0. Never send a driver across cores.
- Embassy time driver (systimer) serves both cores' executors (standard esp-hal
  multicore pattern, [VERIFY] exact init shape against current esp-hal examples).
- CPU clock: esp-hal default (80 MHz) — all hard timing is peripheral-generated;
  [VERIFY] raise to 240 MHz only if director math profile rebuild ever shows up.

---

## 3. Timing Architecture

### 3.1 The frame cycle [ARCH]

One frame = one pass of a 4-one-shot timer chain. The **heartbeat** (timg0.0) fires
**twice per frame** and is the phase-advance authority; two auxiliary one-shots finish
the shutter waveform; the deadman watches the whole chain.

```
t0  ── heartbeat fires: phase=FRAME_START
        · start RMT transaction (non-blocking, preloaded µstep table)   ──► film moves
        · re-arm self for (pulldown + settle)
t1  ── heartbeat fires: phase=EXPOSE_START  (film stationary, settled)
        · shutter FET: LEDC duty 100% (pull-in)
        · arm peak-hold timer (timg1.0) for 4 ms
        · arm exposure-end timer (timg0.1) for exposure_ms
        · re-arm self for (period − pulldown − settle)  → next t0
t1+4ms ── peak-hold ISR: LEDC duty → 25% @ 20 kHz (hold current)
t1+exp ── exposure-end ISR: LEDC duty → 0% (spring closes shutter)
t0'    ── next frame (or: frames_remaining==0 → park, event JobComplete)
```

- Cadence is owned by **timer arithmetic, not by RMT completion** — the pulldown duration
  is deterministic (fixed table per fps), so the phase schedule is computed once per
  fps-change and re-armed blindly. RMT is fire-and-forget.
- Exposure clamp (`period − pulldown − settle − guard`) is enforced when the director
  programs phase durations (SPECS §3.5 table).
- 24 fps example: t0→t1 = 22.9+3 ms, exposure ≤ 15.75 ms, remainder closes the 41.67 ms.
- ISR cost: each firing ≈ 10 register writes, no float, no branches on variables —
  WCET ≪ 10 µs at 80 MHz.

### 3.2 Step generation (RMT) [ARCH]

- **1/16 µstepping [ARCH]**: 30 full steps = 480 µsteps/frame. 480 pulse pairs = 960
  RMT symbols ≈ 1.9 KB DMA buffer (static, in RAM).
- RMT source clock 1 MHz → durations fit the 15-bit symbol field (slowest ramp-start
  µstep ~4 ms < 32.7 ms cap; fastest µstep ~32 µs @ 36 fps boost) [VERIFY] cap vs slowest
  ramp entry).
- Symbol table = **S-curve profile** built by `logic::profile` (trapezoid with
  accel/decel ramps; velocity integral must equal exactly 480 µsteps — unit-tested).
- Per frame: heartbeat ISR calls the **non-blocking** RMT `transmit()` (returns a
  transaction we never `wait()` on). If the channel reports busy at kick time →
  `Fault::RmtBusy` (caught one frame late by design; index watchdog is the backstop).
- Table swap on fps change: director rebuilds into a double buffer, swaps pointer+length
  under `interrupt::free` on core 1 (masks P1; heartbeat is stopped or mid-exposure at
  job boundaries — director only rebuilds between frames, applied at next t0).
- [VERIFY] non-blocking completion/status accessor on the channel; fallback is a direct
  status-register read in the heartbeat ISR (documented register poke if the HAL doesn't
  expose it).

### 3.3 Shutter (LEDC peak-and-hold) [ARCH]

Single LEDC channel (20 kHz) drives the FET: duty 100% = pull-in, 25% = hold, 0% =
release. Pull-in→hold transition is the timg1.0 one-shot (not a task), release is timg0.1.
Fails closed by spring on duty 0 / power loss [LOCKED].

### 3.4 Take-up [ARCH]

LEDC ch1 + DIR GPIO. Duty = feedforward table entry for effective fps. Rate updates
happen at job-arm (director, `arm_with_takeup`) and on the director's 100 ms boost-ramp
tick — **task plane, not per-frame from the heartbeat ISR** (an earlier revision of this
section said "heartbeat refreshes per frame"; that was never true in code and has been
corrected here — audit finding, 2026-09-03). This is a deliberate, accepted deviation
from invariant 1 for this one axis only: takeup tolerance is loose by design (open-loop
feedforward behind a mechanical friction clutch, §3.4/§4.2 SPECS), so task-scheduled
updates are fine. Creep duty in INCH/REWIND. Direction from job. No closed loop in v1
[LOCKED].

### 3.5 Timer inventory [ARCH]

| Timer | User | Mode |
|---|---|---|
| timg0.0 | heartbeat | one-shot, self re-armed (2×/frame) |
| timg0.1 | exposure-end | one-shot |
| timg1.0 | peak→hold | one-shot (4 ms) |
| timg1.1 | esp_rtos core-0 scheduler | executor alarm (P1, infra) |
| systimer unit 0 | embassy time driver | async `Timer::after` (soft plane only) |
| systimer unit 1 | deadman | one-shot, re-armed by heartbeat each firing |

---

## 4. RT Plane Internals

### 4.1 Pure FSM + action records [ARCH]

All ISR logic lives in `logic` crate as pure functions over small structs:

```rust
// logic crate — host-testable, no hardware
pub struct FrameParams { pull_us: u32, settle_us: u32, exp_us: u32, period_us: u32, ... }
pub enum Phase { FrameStart, ExposeStart }
pub struct Actions { rmt_kick: bool, shutter_pull: bool, shutter_hold: bool,
                     shutter_off: bool, arm_heartbeat_us: Option<u32>,
                     frame_done: bool, park: bool }
pub fn advance(phase: Phase, p: &FrameParams, remaining: u32) -> Actions;
```

ISR wrappers in `rt/` apply `Actions` to hardware registers. The FSM is exhaustively
unit-tested on the host, including boundary cases (exposure == clamp, single-frame park,
fps change mid-shot).

### 4.2 Jobs model [ARCH]

The ISR FSM alone isn't enough for multi-second sequences (leader script, rewind,
recover). The RT plane exposes one parameterized **job**, programmed by the director:

```rust
struct Job { direction: Fwd|Rev, rate: Rate,           // Run(fps) | Creep | RewindRate
             frames: Option<u32>,                      // None = until stop
             shutter: Off | Auto,                      // Off for inch/rewind/leader-gap
             counter_target: Option<u32>,              // e.g. rewind to zero
             park_at_end: bool }
```

The ISR decrements `frames`/counter and emits `JobComplete`. Scripts decompose into job
chains issued by the director (leader = 6 frames @6 fps with shutter → advance 10 with
shutter Off → park). This keeps every multi-step sequence deterministic and the director
trivial.

### 4.3 Index watchdog [ARCH — the math]

Perf counting is open-loop [LOCKED]; the index sensor is the step-loss check. 30
full steps/frame vs 200 steps/index → **index edges land mid-frame except every 20th
frame** (lcm(30,200)=600 steps=20 frames). So the watchdog is *arrival-position* based:
heartbeat accumulates commanded full steps; on each index edge the ISR checks
`cumulative ≡ 0 (mod 200)` within ±2 steps; if `cumulative` passes 200+slack with no
edge → `Fault::IndexLoss`. Pure logic in `logic::index_watch`, tested against simulated
drift.

### 4.4 Deadman (last line) [ARCH]

timg1.1 one-shot, re-armed at every heartbeat firing while a job is active; timeout =
2.5 × current period (min 100 ms, set by director per job). On expiry the **P3 ISR
drives safe state directly** (LEDC×2 duty 0, TMC EN low), writes a fault marker to a
`.noinit` static, and stops re-arming — the unfed RTC watchdog then reboots the system,
which boots into `ERROR WD` via the marker. No task participation at any point.

### 4.5 Door interlock ISR [ARCH]

P3 GPIO ISR, debounced by 20 ms timer re-check *inside the ISR chain* (re-arm a one-shot
on first edge, sample again — no task involvement):

- Open → **immediately**: shutter LEDC 0%, take-up 0%, TMC EN low (freewheel), stop
  heartbeat, publish `DoorOpen` event.
- Close → sample state, publish `DoorClosed`. No auto-resume [LOCKED].

### 4.6 Safe-state function [ARCH]

One `rt::safe_state()` (in `#[ram]`): every actuator to its de-energized level. Called
by: deadman, door ISR, panic handler, boot-before-init. The single source of truth for
"everything off".

---

## 5. Command Plane

### 5.1 Tasks [ARCH]

| Task | Core | Loop shape | Owns |
|---|---|---|---|
| `supervisor` | 0 | event-driven (awaits event channel) | interlock matrix, fault latch, RTC-WDT feed, command translation |
| `ui` | 0 | 10 Hz redraw + button events | menus (`logic::menu`), OLED via async I²C, view |
| `power` | 0 | 1 Hz | ADC (VBAT, IPROPI), thresholds 19.8/18.3 V |
| `storage` | 0 | request-driven, **idle-gated** | flash ping-pong, settings codec |
| `director` | **1** | command-driven | TMC5160 (SPI2), jobs, ramps, RMT table, phase params |

Cooperative-executor rule: core-0 tasks must yield within ~30 ms worst case (OLED full
flush ≈ 25 ms @ 400 kHz is the ceiling — acceptable because safety never routes through
core 0). The director on core 1 may block (SPI polling) freely — it has the executor to
itself and ISRs preempt it.

### 5.2 Inter-plane communication [ARCH]

| Direction | Mechanism | Why |
|---|---|---|
| tasks → director | `heapless::spsc::Queue` (cmd, 8 deep) | single producer (supervisor) × single consumer (director), genuinely lock-free |
| ISRs/director → tasks | `embassy Channel<CriticalSectionRawMutex, Event, 32>`, `try_send` | multiple producers; CS cost is µs-scale and bounded; never blocking from ISR |
| status | field-atomic `static Status` (AtomicU32 fields) | single-writer per field; UI reads without locks; no cross-field consistency required for a 128×64 display |

Event enum: `FrameDone(n)`, `JobComplete`, `IndexTick`, `DoorOpen/Close`, `Fault(Fault)`,
`CounterZero`, `SettingsChanged`. Command enum mirrors §4.2 plus `SetFps`, `SetExposure`,
`Boost{on,off}`, `SelfTest`.

### 5.3 Interlocks & faults [ARCH]

The SPECS §11 matrix is implemented **twice**: as ISR-level immediate actions (door,
deadman) and as supervisor-level latching/reporting (everything, including ISR-raised
faults arriving as events). `Fault` enum covers IndexLoss, RmtBusy, DrvStatus(u16),
LowBatt, Deadman, Wdt, Brownout. Latched in `Status` until user ack. The truth table
itself lives in `logic::interlock` and is unit-tested against the SPECS table.

---

## 6. Motion Director (the only task on core 1)

Responsibilities: translate commands into jobs; own TMC5160 (init, mode/current per job,
250 ms status polls while enabled); compute boost ramps (`logic::ramp`, f32 — allowed
here, it's a task); rebuild RMT profiles on fps change; program heartbeat phase params
(integer µs only — **the float→integer boundary is here**); handle brownout-recovery
(`Recover` job: creep to index, park); run self-test.

Rules: never touches outputs the ISRs own during an active job; programs params only at
frame boundaries via a small param mailbox (written under `interrupt::free`, read at t0).

---

## 7. Drivers

| Driver | Mode | Core | Notes |
|---|---|---|---|
| `tmc5160` | blocking SPI (`SpiDevice` + CS) | 1 | register-level, ~200 LOC; init table in consts; StallGuard [FUTURE] |
| `shutter` | LEDC wrapper | ISR-owned | `pull()/hold()/off()` = duty writes |
| `takeup` | LEDC + DIR | mixed | feedforward table |
| `oled` | async I²C (`ssd1306` + `embedded-graphics`) | 0 | full-frame flush 10 Hz max |
| `buttons` | async GPIO + task debounce | 0 | 20 ms re-check pattern |
| `storage` | `embedded-storage` on esp-hal flash | 0 | idle-gated (§12), ping-pong 4 KB, versioned + CRC32 |
| RMT start/status | non-blocking `transmit()` from ISR | 1 | [VERIFY] status accessor |

---

## 8. Global State Model

Top-level states (matches SPECS §8.4): `IDLE · THREAD · RUN · SINGLE · INCH · REWIND ·
SCRIPT · DOOR · ERROR`. Owner: supervisor (authoritative copy in `Status`); director
mirrors job-level state. Transitions: UI commands → supervisor validates against
interlock matrix → command to director → events confirm. UI *displays* state; it never
assumes command success (event-driven display).

---

## 9. Determinism Rules (the law, consolidated)

1. No float in ISRs (Xtensa FPU-in-handler is forbidden/unsupported) — float lives in
   director/tasks; ISRs see integer µs/counts only.
2. No logging, no allocation, no blocking in ISRs. ISR → event → task logs.
3. RT-plane code (`rt/*`, ISRs, `safe_state`) annotated `#[ram]` — immune to flash-cache
   effects during the (idle-only) flash writes.
4. Flash writes only in IDLE/DOOR/parked-ERROR (§12).
5. Single-writer-per-field status; locks only `interrupt::free` and embassy CS mutexes,
   critical sections ≤ µs.
6. Core-0 tasks yield ≤ 30 ms; safety never depends on core 0.
7. All shared ISR/task data is either atomic, spsc, or written under `interrupt::free`
   on core 1 (ISR preemption is the only race on core 1).

---

## 10. Memory & Peripherals

| Resource | Budget |
|---|---|
| RAM | RMT table 2×1.9 KB, OLED buf 1 KB, queues/stacks < 32 KB — ≪ 512 KB |
| PSRAM (N8R2's 2 MB) | **disabled** in v1 — one less nondeterminism source |
| DMA | 1 TX channel for RMT (+SPI2 channel) — S3 has plenty |
| `#[noinit]` | fault marker + last frame counter (brownout forensics) |

---

## 11. Workspace Layout

```
35_motion_camera/
├── SPECS.md · ARCHITECTURE.md
├── Cargo.toml                    # workspace
├── logic/                        # NO hardware, host-testable (cargo test on dev machine)
│   └── src/{lib, profile, ramp, menu, settings, counters,
│            frame_fsm, index_watch, interlock}.rs
└── firmware/                     # esp-hal, no_std, .cargo/config.toml (xtensa target)
    └── src/
        ├── main.rs               # boot: safe-state-first init, planes, executors
        ├── consts.rs             # SPECS-derived: steps/frame=30, timing floors, priorities
        ├── rt/                   # §4 (#[ram] ISR wrappers)
        ├── director/             # §6
        ├── drivers/              # §7
        ├── ui/                   # §5.1
        ├── supervisor.rs · storage.rs · status.rs · fault.rs
```

**Test strategy**:
- Host CI (`cargo test -p logic`): profile integral == 480 µsteps exactly; ramp math;
  index-watchdog drift sims; frame-FSM exhaustive phase/boundary tests; interlock matrix
  == SPECS §11; settings codec round-trip + corruption rejection; capacity math.
- On-target HIL (bring-up firmware, before integration): scope the **debug strobe pins**
  (GPIO 7 reserved: frame-strobe + phase marker driven by heartbeat ISR) to verify: RMT
  kick-to-first-pulse latency, pulldown duration stability, shutter timing vs pulldown,
  door-ISR latency. Acceptance: per-frame jitter < 50 µs non-cumulative.
- `cargo clippy -D warnings`, `cargo size` budget gate.

**Boot order**: `safe_state()` ASAP after entry (outputs safe before peripherals
enabled) → esp-hal init → executors → self-test job (TMC ID read, index present, door
state, battery) → check `.noinit` marker → `IDLE` or `ERROR/RECOVER`.

---

## 12. Deviation from SPECS (needs sign-off)

**Counter persistence**: SPECS §9.4 says "write every 64 frames". Flash erase stalls
cores for ms — violates invariant 1. **This architecture persists counters at idle
boundaries only** (run stop, door events, roll end, settings change). Worst case: a
mid-run power pull loses the current run's frames since last idle event; `Recover`
recovers position ± 1 frame via the index sensor. If losing that count is unacceptable,
the alternative is external FRAM/EEPROM on a spare I²C address (hardware change).

**TMC driver-fault detection (SPECS §11 row "TMC driver fault")**: decision #32's M4c
hardware swap to two TMC2209s in pin mode (STEP/DIR/EN only) retired the SPI/telemetry
layer entirely — there is no DRV_STATUS/StallGuard read path anymore, so this locked
interlock-matrix row currently has **no detection mechanism at all** (not merely
unwired: the signal it would react to no longer exists on this hardware).
`logic::interlock::Condition::DriverFault` and `firmware::supervisor`'s handling of it
are implemented and tested end-to-end for *if* a code ever supplies the condition —
only the sensing side is missing. Needs one of: (a) a current-sense/IPROPI-style proxy
threshold, (b) re-adding a comms channel to at least one axis for periodic status
polling, or (c) formal sign-off that this row is out of scope for v1 pin-mode hardware
(audit finding, 2026-09-03 — flagged, not yet resolved).

---

## 13. Open Verification Items ([VERIFY] — bring-up checklist)

1. ~~esp-hal exact multicore-executor init shape~~ — verified on bench (dual executors, M1+).
2. ~~RMT: non-blocking `transmit()` callable from ISR; completion status accessor~~ — verified on bench (M3a/M3b: RMT kicks from P2 ISR every frame, poll-and-reclaim works).
3. ~~Flash driver cross-core stall behavior~~ — landed and bench-verified (decision #28): writes auto-park core 1; RT-plane code is `#[ram]` so the cache stall is inert; the door ISR is deferred for the write duration (~100-200 ms, idle-boundary only).
4. LEDC 25% hold duty calibration against solenoid measurements (§3.5 SPECS).
5. ~~timg one-shot re-arm latency~~ — verified on bench (24 fps continuous runs; back-to-back re-arms hold frame cadence with no errata).
6. ~~Embassy time driver availability on both cores~~ — verified on bench (timers/executors on both cores, M1+).

---

## 14. Decision Log

| # | Decision | Status |
|---|---|---|
| 1 | Two-plane architecture: ISR/hardware timing, async command plane | [ARCH] |
| 2 | All raw ISRs on core 1; async drivers default P1; P2/P3 for raw | [ARCH] |
| 3 | SPI2/TMC owned by director on core 1; I²C/ADC/buttons on core 0 | [ARCH] |
| 4 | Heartbeat = phase authority (2 firings/frame); RMT fire-and-forget | [ARCH] |
| 5 | 1/16 µstep, 480 µsteps/frame, S-curve table in RAM, DMA | [ARCH] |
| 6 | Peak-and-hold shutter via single LEDC + 2 one-shots | [ARCH] |
| 7 | Deadman ISR as last-line safe state + RTC-WDT reboot chain | [ARCH] |
| 8 | Door interlock entirely in P3 GPIO ISR | [ARCH] |
| 9 | Index watchdog = arrival-position check (lcm math) | [ARCH] |
| 10 | Jobs model for multi-frame sequences; director composes scripts | [ARCH] |
| 11 | No heap; heapless + statics; PSRAM disabled | [ARCH] |
| 12 | Pure FSMs in host-testable `logic` crate; ISR wrappers thin | [ARCH] |
| 13 | Counter persistence idle-only (SPECS deviation, §12) | **needs sign-off** |
| 14 | GPIO 7 = timing debug strobe pins | [ARCH] |
| 15 | Deadman moves to systimer unit-1 alarm (P3); timg1.1 reassigned to the esp_rtos core-0 scheduler. Chip has 4 timg timers; RT plane (4) + executor (1) = 5 needed | signed off 2026-08-29 |
| 16 | Bench strobe pin = GPIO 13 (user LED); final HIL pin TBD | [ARCH] |
| 17 | Deadman (bench, esp32): TIMG1 WDT stage-0 interrupt at P3 (raw registers per IDF `timer_group_reg.h`), fed by the heartbeat each firing, timeout 2.5×period (min 100 ms) programmed at job arm. esp-hal rc has no systimer driver for esp32; the legacy FRC timers are ROM-owned and undocumented. On esp32-s3, revisit #15 (HAL systimer alarm1) or keep the WDT — same register map | signed off 2026-08-29 |
| 18 | Door interlock (§4.5) debounces by timestamp (`Instant::now()`, backed by TIMG0's free-running LACT counter on esp32) rather than a dedicated one-shot timer — all 4 TIMG general timers are already claimed (#15/#17). No door switch on the bench yet; `rt::door::debug_force` exercises the safe-state/event path directly until hardware lands | signed off 2026-08-29 |
| 19 | Bench TMC SPI2 pin map deviates from the SPECS HIL map (CS=10/MOSI=11/MISO=13/EN=14/STEP=15/DIR=16) because the WROVER devkit's GPIO 6-11 (flash) and 16-17 (PSRAM) are not broken out: CS=21, MOSI=23, SCK=12, MISO=13, ENN=14, STEP=15 (RMT), DIR=32. Debug strobe LED (#16) moved from GPIO13 to GPIO27 to free MISO | signed off 2026-08-30 |
| 20 | TMC2240/5160 SPI read quirk: a register read's data is returned one datagram *late* (buffered internally, not same-transaction). `Tmc::read_reg` sends the request then a follow-up transfer to fetch it. Cost a full bring-up session — the symptom (`IOIN=0`, stuck `IFCNT`) was misread as a wiring fault because it was perfectly reproducible, not noisy; confirmed both via the SPI peripheral and an independent bit-bang GPIO probe before the actual cause (protocol timing, not hardware) was found | signed off 2026-08-30 |
| 21 | Film position accounting (§4.2): RT-plane atomic µstep accumulator (`rt::position`), advanced by the heartbeat ISR per counted frame, direction-aware, clamped at the threading datum. Rewind-to-zero = finite reverse job (2 fps, shutter off, `logic::frame_fsm::rewind_params`, below FPS_MIN by design) with frames precomputed from the position — the Job's `counter_target` (ARCHITECTURE §4.2) is materialized by the director as a frame count, keeping the ISR free of target math. `Job` carries `direction`; director sets the TMC DIR pin and the RT direction to match at arm. Park resets the cycle phase to FrameStart (a leftover ExposeStart counted a phantom frame on the next arm). `CounterZero` event at the datum | signed off 2026-08-30 |
| 22 | Boost live-updates go through a *params-only* mailbox applied at FrameStart (frame count/phase untouched); `update_live_params` is a no-op while parked. Without the gate, `SetFps` on a parked transport re-armed the deadman with no heartbeat to feed it → guaranteed watchdog reboot (caught on bench). Full-job arms clear stale live params; `halt()` marks the plane parked so the same class of bug can't recur via the door/Jam paths | signed off 2026-08-30 |
| 23 | Index watchdog bring-up: synthetic edges travel as TEMP commands (`IndexArm`, `IndexEdgeAt`) executed by the director on core 1. Rationale: the watch state is CS-mutex-guarded core-1 data; a core-0 caller held the same mutex concurrently with the core-1 ISR and panicked (`RefCell already borrowed` — critical sections are per-core). Rule going forward: rt CS-mutex statics are touched from core 1 only. The missed-edge check stays silent until the first edge (the film may start anywhere in the sprocket cycle); Jam handling runs inline in the heartbeat ISR because `halt()` self-nests the ISR's own timer lock | signed off 2026-08-30 |
| 24 | Persistence split: `logic::storage_codec` (versioned record + CRC32 + ping-pong 4 KB sector choice, host-tested), `firmware::settings_store` (runtime shadow), idle-gated policy in the storage task behind a `StorageBackend` trait. Bench backend = RAM sectors (validates codec/policy, not non-volatile). The real esp-hal flash backend targets the partition-table data region next (see §13 item 3) | signed off 2026-08-30 |
| 25 | TMC S2G/OL status bits (S2GA/S2GB/OLA/OLB) false-trip on the bench at creep speeds, standstill, and IHOLD (TMC app-note behavior). Policy: hard-fault mask = OT|OTPW only; S2G/OL reported as on-change log advisories. Revisit with final motor wiring at HIL | signed off 2026-08-30 |
| 26 | Event-channel discipline (extends #23): the embassy `Channel<CriticalSectionRawMutex>` is per-core-CS and drops cross-core sends — high-rate frame events (FrameDone/FrameExposed at 24+ Hz) now bypass the channel entirely: the heartbeat ISR writes `status::STATUS` atomics directly and the supervisor polls at 20 ms. Only rare events (JobComplete, CounterZero, door, fault, index tick) stay on the channel, where loss is tolerated or backstopped. Also: the boost ramp must rebuild the RMT step table in lockstep with live params — a live cadence change without a table rebuild overruns the frame transfer (RmtBusy, caught on bench) | signed off 2026-08-30 |
| 27 | Film-end wiring + Recover (SPECS §11): `Command::Recover` creeps forward with `frames: None` and parks on the next accepted index edge (the edge path requests the park; bench-validated with synthetic edges). Film-end = supervisor-side check on the polled exposure counter against `settings.roll_frames` → interlock FilmEnd → `Stop` command. Note: the esp-rtos core-0 embassy executor quantizes task polling to ~100 ms (observed: 20 ms timeouts fire at ~100 ms), so the counter poll and roll-end stop are frame-accurate but not millisecond-accurate — acceptable for v1, revisit with the UI milestone | signed off 2026-08-30 |
| 28 | Real flash persistence: `esp-storage` 0.10 `FlashStorage` (read_nor/write_nor/erase) on the first two 4 KB sectors of the `nvs` partition region (flash 0x9000/0xA000; the IDF boot log confirms a 24 KB `nvs` partition there and nothing else uses it — no partition-table API needed, fixed addresses). `multicore_auto_park` parks core 1 for the erase+program (~100-200 ms, idle boundaries only); the door ISR is deferred for the write duration — the flash-on-same-bus tradeoff from #13, now quantified. Record survives reflash (espflash erases only app partitions). Bench-verified over 7 reboot cycles: seq monotonic, sectors ping-pong, boot load correct | signed off 2026-08-30 |
| 29 | Bench UI pin map (M4a): LCD 1602A over PCF8574 backpack on I2C0 SDA=GPIO18/SCL=GPIO19 (21/22 are taken by TMC CS / nothing); buttons RUN=GPIO5, MENU=GPIO25, ▲=GPIO26, ▼=GPIO22 (was GPIO33 — unreliable on this devkit), active-low with internal pull-ups | signed off 2026-08-30 |
| 30 | FPS pot input (bench): B10K as a divider on GPIO34 = ADC1_CH6 (input-only pin, no pull needed), 11 dB attenuation, polled at 100 ms. Raw quantized into whole-fps buckets 3..=36 (same full-step model as the menu adjust — the earlier 0.5-step f32 setting was fake precision: every consumer rounded to integers, so display and motor disagreed). A `UiEvent::SetFps` fires only when the bucket changes after two agreeing samples (no boundary flicker) and the supervisor dedupes against the current setting, so an idle pot never fights menu edits. FPS changes apply to settings + the next arm; a running take keeps its armed cadence (only boost ramps are live-updatable — #26) | signed off 2026-08-30 |
| 31 | Shooting cluster + second pot (M4b): BOOST=GPIO33 (internal pull-up), FRAME=GPIO36, INCH=GPIO39 (input-only — bench wires external 10k pull-ups on 36/39); DOOR stays on GPIO4, bench-wired as a momentary button (hold = closed). Exposure pot on GPIO35 = ADC1_CH7, sharing the ADC1 unit and deadband-hysteresis pattern with the fps pot (2..=1000 ms whole steps). Semantics per SPECS §9.2: BOOST hold = live boost ramp (ignored when parked — boost is a sub-state of RUN §4.2); FRAME = one-frame Run when parked; INCH hold = Inch{MAX} until release sends Stop (frame-boundary park). Shooting-cluster buttons are always live, from any menu level (like RUN) | signed off 2026-08-30 |
| 32 | Hardware swap to two TMC2209s + real shutter + spec OLED (M4c): (1) **Both axes in pin mode** — STEP/DIR/EN only; MS jumpers (16 µsteps) + Vref on the boards; the SPI/telemetry layer (tmc.rs: two-transaction read quirk, DRV_STATUS polling, S2G/OL advisories) is retired, so there is no comms self-test or fault polling anymore. Transport keeps EN=14/STEP=15/DIR=32; takeup reuses the freed SPI pins EN=12/STEP=23/DIR=21. (2) **Takeup = LEDC pulse train**: HS timer 1 + channel 1, 10-bit duty at 50%, raw-register rate changes from the director; feedforward ≈ 363 µsteps/frame (`TAKEUP_USTEPS_PER_FRAME`, 20 mm spool core — ballpark, the friction clutch is the real tension loop). (3) **Real shutter**: the LEDC channel 0 output drives a MOSFET gate (active-high — the bench inversion is gone); solenoid + 1N4001 flyback + 10k gate pulldown per the wiring guide; peak-and-hold duty table unchanged (100/25/0). (4) **SSD1306 OLED 128×64** (ssd1306 0.9 + embedded-graphics 0.8, FONT_8X13, addr 0x3C/0x3D) replaces the retired 1602A — same two-line UI render, presentation-agnostic. (5) Fixed a latent re-enable bug found during the swap: safe_state() pulls both ENNs high, but nothing re-enabled the drivers afterwards — `arm_with_takeup` now re-enables both axes on every job arm | signed off 2026-09-02 |
| 33 | Full-codebase audit + fix pass (2026-09-03), no hardware/spec changes: (1) takeup LEDC divisor's Q8 fixed-point scale (#0c6a8b1) had been reverted (#8fb88fa) without a follow-up fix — re-applied; the bug was live at HEAD (256x too fast). (2) `heartbeat::halt()` (door/jam callers) never reset `Cycle::phase`, generalizing the decision-#21 phantom-frame bug to every halt() path, not just the voluntary park — now forces `FrameStart` unconditionally. (3) `rt::arm_job`'s "only between jobs" precondition was enforced for Run/Inch/Frame but not the menu's TRANSPORT actions (Rewind/LeaderMark/TrackBSetup) — added the same `is_parked()` gate at both the supervisor and director call sites. (4) Low-battery interlock: only one ADC threshold existed (mapped to the wrong ErrorCode) and `Event::Fault` never actually stopped anything — `power.rs` now raises both SPECS §11 thresholds (19.8 V warn-only, 18.3 V critical), and the supervisor's fault handler now runs every fault through `logic::interlock::evaluate()` and acts on `stop_now`/`stop_at_frame_end`/`persist`, not just the display code. (5) Added `fault::raise()` as the actual backstop decision #26 already claimed existed: writes `status.fault` directly (not only the lossy cross-core channel) for every raised fault, matching the pattern already used for frame counters; `RunToggle` now blocks on a nonzero latched fault and treats the next RUN press as the acknowledgement SPECS §11 requires (there was no ack path at all before). (6) The `.noinit` boot fault marker was logged but never surfaced into `Status` — a watchdog-reboot now correctly shows `ERR` at boot instead of a clean `IDLE`. (7) `RECOVER_PENDING` could leak from an aborted/interrupted Recover job into an unrelated later job and spuriously stop it — cleared on the jam/misalign fault path and on every `arm_job`. (8) Door debounce swallowed the true open transition when a bounce burst's first sample happened to read "closed" — rebiased so any "open" reading is accepted immediately (fail-safe direction), only "closed" readings go through the spacing debounce. (9) The persisted exposed-frame counter (the only film-end safety mechanism per SPECS §5 — no physical end-of-roll sensor) was never restored at boot, silently resetting to 0 on every reboot, not just the documented idle-boundary loss (§12) — supervisor now seeds it from the flash-loaded payload. (10) Added `liveness` module + wired it into `wdt.rs`: the RTC-watchdog feed was gated only on `rt::safe_active()`, so a command-plane task hung-but-still-yielding was invisible to it; now every command-plane task's per-loop counter must advance within a 3 s window. (11) `ui_task`'s `.expect("i2c init")` could panic the whole core-0 executor (same blast radius as a supervisor panic) — now degrades to headless mode like every other OLED failure path already does. (12) Minor: removed the stale `DEBUG_STROBE_GPIO` constant (numerically collided with the M4c shutter-gate pin after #32's pin-map change, though never wired); `storage_codec::choose()`'s sequence comparison is now wraparound-safe; `Settings::is_valid()` now range-checks the ramp-rate fields; stale doc text in §3.1/§3.4 claiming per-frame heartbeat-driven takeup updates corrected to match the actual task-plane (job-arm + boost-tick) implementation. (13) TMC driver-fault detection (SPECS §11 row) has no sensing mechanism since #32 retired SPI telemetry — flagged as an open deviation in §12, not fixed here (requires a hardware or scope decision, not a code fix) | signed off 2026-09-03 |
| 34 | ESP32-S3 migration (chip swap after the classic ESP32 died on the bench): toolchain/features flip to `xtensa-esp32s3-none-elf` (all deps ship esp32s3); logging moves to native USB-Serial-JTAG (`esp-println jtag-serial`). Hardware realities that forced code/board changes: **GPIO22-25 don't exist on the S3** and **GPIO33-37 are octal PSRAM on the N16R8 module** — MENU/▲/▼ moved 25/26/22→26/29/28, BOOST 33→48, FRAME 36→12; pots moved 34/35→1/2 because the S3's ADC1 lives on GPIO1-10 (34/35 aren't ADC pins); OLED SCL 19→17 (GPIO19 = USB D−); takeup STEP 23→38. FRAME/INCH are now always-on (S3 has internal pull-ups on every pin — the external-pull-up feature gate is retired). LEDC: the S3 is the low-speed variant — `LowSpeed`/`LSClockSource` in the HAL, `timer()`/`ch()` register accessors, `clk_div` (not `div_num`), `para_up` latch pulses, and the fade-free `conf1` start sequence in raw writes. esp-hal time on the S3 is SYSTIMER-based, so the classic-ESP32 LACT/`reinit_hal_clock()` workaround is deleted outright. `rt::deadman` classic raw TIMG1-WDT pointers (0x3FF60000) are invalid on the S3 (TIMG1 = 0x60020000, per-stage `WDTCONFIG[n].hold`) — rewritten next in the migration series as a 2-stage WDT. RMT: the S3 has 192 words of RMT RAM (4×48) vs the classic's 512 — the 481-entry step table no longer fits whole, so the transport STEP generator moves to an ISR-driven mid-transfer refill design | signed off 2026-09-03 |
| 35 | S3 deadman rewrite (completes #34): TIMG1 MWDT through the typed esp32s3 PAC registers (0x60020000, per-stage `WDTCONFIG[n].hold`, prescale 80 = 1 µs ticks, key 0x50D83AA1) as a **two-stage** watchdog. Stage 0 = interrupt at the programmed 2.5×period timeout (P3 ISR: latch safe state, write fault marker, disarm); stage 1 = hardware reset at 2× that as the new backstop for a dead interrupt path — the classic ESP32 deadman had no such escalation | signed off 2026-09-03 |
| 36 | S3 transport STEP refill (completes #34): the 481-entry step table no longer fits the S3's 192-word RMT RAM, so the transfer now streams — esp-hal preloads the first 192 entries and a dedicated P2 RMT-threshold ISR (bound in `rmt_step::init`, channel 0) refills the FIFO from the prebuilt table via `TxTransaction::poll()` while the tail plays. Threshold = half the channel RAM (96 words); worst-case drain at 36 fps is ~31 entries/ms, giving the ISR a ~3 ms runway — refill cannot miss unless interrupts are dead, in which case the stall raises the usual RmtBusy fault. Refill count published as `status.rmt_refills` for bench verification | signed off 2026-09-03 |
| 37 | OLED I2C auto-recovery: a wedged bus no longer degrades to permanent headless. A failed flush starts a 2 s cooldown; after that the UI retries with the same bus instance — esp-hal's blocking driver runs its bus-clear sequence (`ensure_idle_blocking` → `clear_bus`) before every transaction, so transient wedges (motor noise, display glitch) self-heal. Each attempt is bounded by the 300 ms software transaction timeout, so a hard-stuck bus costs ≤300 ms of core-0 blocking every 2 s. Faults counted under debug-prints | signed off 2026-09-03 |
