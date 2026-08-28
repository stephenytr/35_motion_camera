# 35mm 1.5-Perf Motion Picture Camera — Control Electronics & Firmware Specification

**Document**: SPECS.md · v1.0 · 2026-08-28
**Project**: 35_motion_camera

**Legend**:
- **[LOCKED]** — decision confirmed by project owner. Do not change without sign-off.
- **[SPEC]** — proposed by this document (delegated or open). Review before implementation.
- **[FUTURE]** — out of v1 scope, but the v1 design accommodates it.
- **[OOS]** — out of scope for this document (mechanical fabrication, bench validation).

---

## 1. System Overview

A battery-powered, 1.5-perf 35mm motion picture camera built around an ESP32-S3 running
bare-metal Rust (esp-hal). Film advance by direct-drive stepper + custom half-perf-pitch
sprocket. Single-blade solenoid shutter. Take-up by geared DC motor with mechanical slip
clutch. 135 still-cartridge film supply (manual roll-length entry, no DX contacts).

```
                6S LiPo (18.0–25.2 V)
                    │
        ┌───────────┼──────────────┬─────────────────┐
        │           │              │                 │
   TMC5160      Solenoid       DRV8871          36V buck → 5V → 3.3V
   (stepper)    N-FET+TVS      (take-up)            │
        │           │              │                 │
   NEMA14 +     Shutter       N20 gearmotor    ESP32-S3-WROOM-1
   20T sprocket blade+spring  + slip washer          │
        │                                         ┌──┴───┐
     INDEX                                      I2C OLED  7 buttons,
   photointerr.                                SSD1306   door switch, mode switch
                                              ADC: VBAT, IPROPI
```

---

## 2. Film Transport

### 2.1 Film & format constants [LOCKED with values SPEC]

| Constant | Value | Note |
|---|---|---|
| Film width | 34.98 ± 0.03 mm | 135 type (ISO 1007) |
| Perforation | KS-1870, pitch **4.750 mm** | perf 2.794 mm long × 1.854 mm tall |
| Frame pitch (1.5 perf) | **7.125 mm** | 140.3 frames per metre |
| Pulldown per frame | Constant in both track modes | gate/mask difference only [LOCKED] |
| Modes | whole-width / two-track | detected by SPDT switch, read at idle [LOCKED] |

### 2.2 Sprocket [SPEC — geometry; fabrication OOS]

| Parameter | Value |
|---|---|
| Type | custom, **half-perf tooth pitch = 2.375 mm** |
| Teeth | **20** |
| Pitch diameter | 15.10 mm |
| Outside diameter | ~17.7 mm |
| Circumference | 47.50 mm = 10 perfs/rev |
| Tooth width | 1.30 ± 0.03 mm (clears inter-perf gap 1.956 mm; ~0.75 mm/side play in perf) |
| Mount | direct on stepper shaft, no belt/gear |

**Rationale**: perf length 2.794 > tooth pitch 2.375 → 1–2 teeth engaged in every perf at
all phases → continuous positive engagement. Every half-perf multiple = integer teeth.

### 2.3 Stepper motor [SPEC — chosen from comparison]

| Parameter | Spec |
|---|---|
| Frame | **NEMA 14** (35×35 mm), body 28–40 mm |
| Step angle | 1.8° (200 steps/rev) |
| Winding | low inductance ≤ 3 mH, 2–3 Ω/phase |
| Run current | 1.0–1.2 A/phase (set via TMC5160 SPI) |
| Mass | 140–220 g |
| Demand at 24 fps | ~0.035–0.05 N·m at ~1,750 full-steps/s (inertia + 1–2 N film tension × 7.6 mm radius + margin) |

Rejected: NEMA 8 (underpowered), NEMA 11 (marginal, boost-limited), NEMA 17 (overkill).
**Firmware fps ceiling [SPEC]: 36 fps** (48 fps needs NEMA 17 — [FUTURE]).

### 2.4 Steps-per-frame (the core firmware constants)

| Quantity | Value |
|---|---|
| Steps per tooth | **10 full steps** (18°) |
| **Steps per frame, 1.5 perf (3 teeth)** | **30 full steps = 54° = 480 ×1/16 µsteps = 960 ×1/32 µsteps** |
| Frames per sprocket rev | 6⅔ (frame phase is threading-relative; sprocket-only registration [LOCKED]) |

Variable-perf table [FUTURE — same sprocket, firmware constant only]:
1 perf = 20 steps · 1.5 perf = 30 · 2 perf = 40 · 3 perf = 60 · 4 perf = 80.

### 2.5 Index sensor [SPEC]

Slotted photointerrupter (ITR9608 / EE-SX1102 class), 3.3 V, one slot in sprocket hub,
1 pulse/rev → GPIO interrupt. Perforation counting is stepper-open-loop [LOCKED]; the
index sensor's job is a **step-loss watchdog**: expect an edge every 200 full steps;
deviation ⇒ jam/missed-step fault (§11). Power-up homing (optional creep ≤1 rev @ 0.5 fps).

### 2.6 Registration & holding [LOCKED + SPEC]

- Sprocket is the only registration in v1. Registration-pin solenoid is [FUTURE].
- While filming: TMC5160 stays enabled, **IHOLD = 30%** (carries ~2 N take-up tension
  through the sprocket during exposure without heating).
- Idle / door open / error: driver disabled (EN low, freewheel) so film can be handled.

### 2.7 Loops [LOCKED]

No loop formers, no loop sensors. Transport is cartridge → gate → sprocket → take-up,
straight-line/short path. Loop-loss handling: N/A in v1 (jam watchdog covers derangement).

---

## 3. Shutter

### 3.1 Blade & linkage [SPEC — design targets; verify at integration, bench OOS]

| Parameter | Spec |
|---|---|
| Type | single pivoting blade sweeping the gate aperture |
| Blade | 0.20 mm spring-tempered 301 stainless, ≤ 1.5 g, sized to cover ~36 × 10 mm aperture region |
| Sweep | 30° (0.52 rad) |
| Pivot | Ø3 mm pin, two bushings, axis parallel to film plane, clear of aperture |
| Crank | 7 mm radius on pivot shaft; solenoid plunger → crank pin |
| Stroke at crank | ~3.6 mm → spec solenoid stroke 4 mm |
| Assembly inertia | ≤ 8 × 10⁻⁷ kg·m² |

### 3.2 Return spring [SPEC]

| Parameter | Spec |
|---|---|
| Type | stainless torsion spring on pivot shaft |
| Rate | **k ≈ 0.06 N·m/rad** |
| Preload (closed) | ≈ 0.015 N·m |
| Closing time | ≈ 3.7–5 ms (spring-driven, I ≤ 8×10⁻⁷) |
| Fail state | **fails closed** (spring) on any power loss [required] |

### 3.3 Solenoid coil [SPEC]

| Parameter | Spec |
|---|---|
| Type | pull-type tubular, Ø15–16 × 25–30 mm, spring return, link/lever to blade |
| Voltage | 24 V class, driven from 6S rail |
| Coil resistance | **20 Ω ±10%** → 1.3 A @ 25.2 V, **0.9 A @ 18.0 V (worst-case battery)** |
| Force | **≥ 12 N @ 4 mm stroke** (opens blade + spring in ≤ 5 ms incl. dynamics) |
| Timing | pull-in ≤ 5 ms @ 18 V; release ≤ 5 ms |
| Duty | peak 100% for 3–4 ms, then **25% PWM @ 20 kHz hold (~0.3 A)**; ≥ 3.5 W continuous rating |

### 3.4 Driver circuit [SPEC]

- 60 V logic-level N-FET, low side: IRLZ44N (TH) or AOD4184 (SMD); 100 Ω gate, 10 k pull-down.
- **TVS SMAJ33A across coil for decay** — deliberately *not* a rectifier flyback diode
  (diode current decay is too slow; it would eat the minimum exposure time).
- LEDC hardware PWM for the hold phase; FET fully on during pull-in transit.

### 3.5 Exposure semantics & timing budget [SPEC]

- Exposure = total aperture-open time (transit in + hold + transit out), user setting
  2–1000 ms, automatically clamped to the frame window.
- Minimum clean full-open ≈ 10 ms (2 × ~4 ms transit + hold).
- Per-frame phases: **PULLDOWN → SETTLE (3 ms) → EXPOSE → (next)**.
- Derived limits (55% pulldown window, 3 ms settle):

| FPS | Period | Max exposure | ≈ Shutter angle |
|---|---|---|---|
| 3 | 333 ms | ~147 ms | 159° |
| 12 | 83.3 ms | ~34.5 ms | 149° |
| 24 | 41.7 ms | ~15.7 ms | 135° |
| 36 | 27.8 ms | ~9.5 ms | 123° |

- Open-loop timing in v1 (calibrated constants in settings). Blade position sensor: [FUTURE].
- Flash-sync contact output: not in v1 [FUTURE].

---

## 4. Motion Control

### 4.1 Frame cadence [SPEC]

One hardware timer owns the cadence; per-frame state machine executes the phases of §3.5.
Step generation: **RMT TX channel (DMA-capable) preloaded with the per-frame µstep timing
table** → hardware-timed trapezoid/S-curve, zero CPU jitter. Fallback: timer-ISR-per-pulse.
All rate changes apply at frame boundaries only.

### 4.2 FPS, boost, single-frame, inching [LOCKED semantics, SPEC parameters]

| Function | Spec |
|---|---|
| FPS range | **3.0–36.0 in 0.5 steps**, menu-set. No crystal sync (ordinary clock, ±hundreds ppm OK) [LOCKED] |
| Boost | momentary hold; target = **1.5 × current fps, clamp 36**; ramp **up 24 fps/s, down 48 fps/s** (all three menu-configurable) [LOCKED momentary; SPEC numbers] |
| Single frame | edge-triggered, one full cycle (pulldown + expose), parks shutter closed; ignored while a cycle is running [LOCKED] |
| Inching | hold-to-creep at 1 fps, shutter locked closed, low current, take-up at creep duty; release = stop [LOCKED] |
| Stop/park | shutter closed, mechanism at frame boundary |
| Power-fail mid-frame | see §11 |

---

## 5. Take-Up

| Parameter | Spec |
|---|---|
| Motor | **N20-class brushed DC gearmotor, metal gears, 12 V winding, 100–300 RPM output** [SPEC] |
| Drive | from 6S rail, PWM duty-capped ~55% |
| Running current | 60–150 mA nominal |
| Control | PWM feedforward table vs fps; slip washer is the torque limiter [LOCKED mechanical] |
| Driver | **DRV8871 H-bridge** (45 V), IPROPI current-sense → ADC; direction pin enables REWIND (§10) and future bulk spool [SPEC] |
| Speed demand | 100–275 RPM across spool Ø12→32 mm (≤171 mm/s film speed) |
| Film-end detection (v1) | frame counter only [LOCKED] |
| Film-end / jam (v1.x) | IPROPI elevated-current threshold + index watchdog [FUTURE] |

---

## 6. Sensors & Interlocks

### 6.1 Door switch [LOCKED: detect-only; SPEC implementation]

- **Hall switch DRV5032 + magnet in door** (no alignment-critical contact). Fallback: roller microswitch.
- Fail-safe wiring: door closed = switch pulls line **low**; open *or broken wire* = high
  (internal pullup + 100 nF, interrupt on both edges, 20 ms debounce).
- Interlock matrix: §11.

### 6.2 Other inputs [LOCKED/SPEC]

| Input | Spec |
|---|---|
| Mode switch | SPDT slide, "FULL / DUAL", GPIO, debounced, read only while idle, persisted in settings [LOCKED] |
| Roll length | manual entry: presets 24-exp / 36-exp equivalents, metres, or custom frame count. No DX contacts [LOCKED] |
| Battery monitor | 100k:10k divider → ADC, warn ≤ 19.8 V, auto-stop ≤ 18.3 V [SPEC] |
| Cartridge presence | none in v1 [OOS] |

---

## 7. Electrical

### 7.1 Power tree [SPEC]

| Rail | Source | Consumers |
|---|---|---|
| V_MOT | 6S direct | TMC5160 VM |
| V_SOL | 6S direct | shutter FET |
| V_TAKE | 6S direct | DRV8871 |
| 5 V | 36 V-rated sync buck (LMR36015-class), 2 A | 3.3 V LDO, OLED |
| 3.3 V | LDO from 5 V | ESP32-S3 module, sensors, logic sides |

- Input: 3–5 A fuse + P-FET reverse polarity protection.
- VM bulk 470 µF electrolytic + 100 nF ceramics at each driver; SMAJ33A TVS across VM.
- Budget ≈ 10–14 W avg at 24 fps → 6S 1000 mAh ≈ 1.5–2 h mixed use.
- Brownout: see §11.

### 7.2 Stepper driver [SPEC: **TMC5160**]

| Requirement | TMC5160 | Notes |
|---|---|---|
| VM range | 8–60 V | 25.2 V pack + regen headroom (TMC2209's 29 V ceiling rejected) |
| Interface | SPI for config/status; STEP/DIR from RMT | current set in register — no trim pot |
| Diagnostics | StallGuard2, full status | jam detection [FUTURE], OTFLAG |
| Settings | IRUN 1.0–1.2 A, IHOLD 30% while filming, freewheel when idle/door open | §2.6 |

---

## 8. MCU & Firmware Architecture

### 8.1 Chip & toolchain [LOCKED]

- **ESP32-S3-WROOM-1 N8R2** (devkit for prototyping; module for final).
- Xtensa LX7 dual core: toolchain via `espup`, target `xtensa-esp32s3-none-elf`.
- **esp-hal 1.0, no_std + embassy** (async tasks for UI/menus/supervisor; hard timing in
  RMT/ISRs). Logging + flashing over **USB-Serial-JTAG** (no debug probe needed).

### 8.2 Peripheral allocation [SPEC]

| Function | Peripheral | Notes |
|---|---|---|
| Stepper STEP | RMT TX (DMA channel) | preloaded per-frame profile |
| Solenoid hold PWM | LEDC ch0, 20 kHz | FET gate |
| Take-up PWM | LEDC ch1, ~20 kHz | DRV8871 |
| TMC5160 | SPI2 | config/status/current |
| OLED | I2C0, addr 0x3C | SSD1306 |
| Door, index, mode, 7 buttons | GPIO interrupts | |
| VBAT, IPROPI | ADC1 | dividers per §6 |
| Watchdog | RTC WDT + task WDT | forced safe state + reboot |

### 8.3 Pin map [SPEC]

| GPIO | Function | | GPIO | Function |
|---|---|---|---|---|
| 1 | ADC: VBAT divider | | 12 | SPI SCK (TMC) |
| 2 | ADC: IPROPI (take-up current) | | 13 | SPI MISO |
| 4 | DOOR switch (IRQ) | | 14 | TMC5160 EN |
| 5 | Sprocket INDEX (IRQ) | | 15 | STEP (RMT) |
| 6 | MODE switch | | 16 | DIR |
| 8 | I2C SDA (OLED) | | 17 | SOL PWM (LEDC0) |
| 9 | I2C SCL | | 18 | TAKE PWM (LEDC1) |
| 10 | SPI CS (TMC) | | 21 | TAKE DIR (DRV8871) |
| 11 | SPI MOSI | | 38–42, 47, 48 | RUN, BOOST, FRAME, INCH, MENU, UP, DOWN |

Reserved: 19/20 (USB-JTAG), 0/3/45/46 (strapping), 26–37 (module flash/PSRAM).

### 8.4 Firmware structure [SPEC]

- Top-level states: `IDLE · THREAD · RUN · SINGLE · INCH · BOOST(sub-of-RUN) · REWIND ·
  DOOR · ERROR`.
- Tasks: `timing` (timer ISR + RMT supervisor), `transport` (TMC SPI), `shutter`,
  `takeup`, `ui` (buttons + OLED + menus), `supervisor` (interlocks, watchdog, power),
  `storage` (settings/counters, §9.4).
- Constants live in one module (`consts.rs`): steps-per-frame table, phase timing,
  fps limits, ramp rates — everything derived from §2.4.

---

## 9. User Interface

### 9.1 Display [SPEC]

0.96" **SSD1306 128×64, I²C** (`ssd1306` + `embedded-graphics` crates).

### 9.2 Buttons [SPEC]

| Button | Type | Function |
|---|---|---|
| RUN | 12 mm momentary | toggle run/stop |
| BOOST | 12 mm momentary | hold = boost (§4.2) |
| FRAME | 6 mm momentary | single frame |
| INCH | 6 mm momentary | hold = inch |
| MENU | 6 mm | enter/leave menu, select |
| ▲ / ▼ | 6 mm | adjust; long-press = back |

All active-low, internal pullups, 20 ms debounce.

### 9.3 Physical layout [SPEC — proposal]

Rear panel, landscape orientation:

```
┌──────────────────────────────────────────────┐
│  ╔══════════════╗                             │
│  ║   OLED       ║                ┌────────┐   │
│  ║  128×64      ║                │  RUN   │   │
│  ╚══════════════╝                │  12mm  │   │
│                                  └────────┘   │
│   ┌────┐                         ┌────────┐   │
│   │MENU│      ▲                  │ BOOST  │   │
│   └────┘     ┌─┐                 │  12mm  │   │
│              │▼│                 └────────┘   │
│   FRAME    INCH                              │
│                                             │
└──────────────────────────────────────────────┘
```

- OLED top-left; menu cluster (MENU, ▲, ▼) left, single-hand reach.
- Shooting cluster right: RUN top (thumb), BOOST directly below (index), FRAME + INCH
  bottom row. Shooting buttons ≥ 18 mm centre spacing (gloved use).
- Mode switch: recessed slide on top edge near door hinge, labelled FULL / DUAL.
- All controls on one ribbon/header to main PCB; labels engraved on panel.

### 9.4 Main screen & menus [SPEC]

Main screen (idle/running):

```
┌────────────────────────┐
│ 24.0fps  1.5P  FULL    │   fps · perf mode · track mode
│ FR 0142/0229           │   frame counter / roll capacity
│ EXP 12.4ms  BAT 24.1V  │   exposure · battery
│ ► RUNNING              │   state line (errors override)
└────────────────────────┘
```

Menu tree: `FPS · EXPOSURE · ROLL (length) · TRACK (A/B, DUAL only) · BOOST (target,
ramp up, ramp down) · TRANSPORT (Track Leader / Rewind / Track B Setup — §10) ·
SETTINGS (hold current, index cal, battery cal) · ABOUT`.

Counters:
- FULL mode: capacity = length ÷ 7.125 mm (36-exp ≈ 1.63 m ≈ **229 frames ≈ 9.5 s @ 24 fps**).
- DUAL mode: 229/track, two counters (A/B), active track = switch + menu; footage math
  identical (pulldown unchanged [LOCKED]).

Persistence: internal flash, ping-pong 4 KB sectors; write on settings change
(debounced), door events, roll end, every 64 frames. (~230 frames/roll ⇒ negligible wear.)

---

## 10. Two-Track Operation & Leader-Marking Procedure [SPEC]

Registration is sprocket-only [LOCKED], so track-to-track alignment is procedural:

**Threading datum rule (both passes)**: a scribed datum line in the gate aligns with
sprocket tooth 0 (the index-sensor tooth). Thread with **a perf edge seated on the datum**,
under take-up creep tension (0.5 fps creep during threading) so the perf always seats on
the same tooth flank. Tolerance ±0.3 mm.

**Pass 1 — Track A:**
1. Set mode DUAL, TRACK A, enter roll length, zero counters.
2. Thread per datum rule.
3. Menu → `TRANSPORT → TRACK LEADER`: exposes 6 dense frames @ 6 fps at a bright
   uniform target (leader marks), then advances a 10-frame gap (lens capped), then
   parks. Shoot track A normally.

**Rewind:**
4. Menu → `TRANSPORT → REWIND`: stepper reverse @ 2 fps equivalent, DRV8871 reverse
   at ~20% duty, shutter locked closed, door closed, counter decrements to zero. Stop
   manually if partial rewind wanted.

**Pass 2 — Track B:**
5. Move lateral film guide to TRACK B position; re-thread per datum rule.
6. Menu → `TRANSPORT → TRACK B SETUP`: advances (6 + 10 + frames-used-on-A) × 30 steps
   at creep speed, placing track B start at pass 1's start position.
7. `TRACK LEADER` for B, then shoot.

**Verification (post-development QC)**: pass-2 leader frames should sit on pass-1 marks
within ±1.5 mm (±0.3 perf). Error of k perfs ⇒ datum slip on rethread; log it, adjust
technique. The 10-frame gap gives a clean inspection zone.

Bulk roll spool: [FUTURE] — changes take-up torque profile only; DRV8871 direction +
IPROPI already support it.

---

## 11. Failure Handling [SPEC]

### Interlock matrix

| Condition | Detection | Immediate action | Recovery |
|---|---|---|---|
| Door open | switch IRQ | solenoid off (spring closes shutter) → stepper EN low → take-up off → abort run/single/inch → persist counters+settings → `DOOR_OPEN` on display | explicit RUN after close; no auto-resume |
| Jam / missed steps | index watchdog (edge ≠ every 200 full steps) | stop cadence, shutter closed, take-up off, driver fault readout | `ERROR JAM`, user ack |
| TMC driver fault | SPI status (OT, short) | as above | `ERROR DRV:<code>` |
| Low battery | ADC | warn ≤ 19.8 V on display; auto-stop ≤ 18.3 V (finish current frame, park) | recharge |
| Watchdog / task hang | RTC WDT + task WDT | solenoid off, EN low, take-up off, reboot to safe state | boot → `ERROR WD` |
| Brownout mid-frame | POR/BOD | shutter spring-closes; on reboot offer `RECOVER`: step to next frame boundary via index sensor before anything else | user ack |
| Film end | counter = capacity | stop, park, `ROLL END` | new roll procedure |

Power-on: self-test (driver status, index present, door state, battery), then `IDLE`,
shutter closed. Errors latch until user acknowledgement.

---

## 12. Scope Fence

**v1**: everything in this document.
**[FUTURE]**: bulk spool, variable perf (table already fixed §2.4), registration-pin
solenoid, 48 fps / NEMA 17, flash-sync output, blade position sensor, StallGuard jam
detect, WiFi/BLE remote (would revisit chip/stack choice).
**[OOS]**: sprocket fabrication details, shutter blade fabrication, bench validation of
§3.1–3.3 dynamics (numbers herein are design targets to verify at integration).

---

## 13. Decision Log

| # | Decision | Status |
|---|---|---|
| 1 | Half-perf 20T sprocket, direct-drive stepper | [SPEC] |
| 2 | NEMA 14, 1.8°, ≤3 mH, 1.0–1.2 A | [SPEC] |
| 3 | 30 full steps/frame @1.5 perf | derived [SPEC] |
| 4 | Sprocket-only registration | [LOCKED] |
| 5 | No loops | [LOCKED] |
| 6 | Stepper-only perf counting + index watchdog | [LOCKED] |
| 7 | Solenoid shutter, fails closed, peak+hold | [LOCKED concept / SPEC numbers] |
| 8 | 3–36 fps, momentary boost w/ ramp, single-frame, inching | [LOCKED semantics / SPEC params] |
| 9 | N20 12V take-up + DRV8871 + slip washer | [LOCKED mechanical / SPEC motor] |
| 10 | Door = detect switch, fail-safe wiring | [LOCKED] |
| 11 | Manual roll entry, no DX | [LOCKED] |
| 12 | 6S LiPo | [LOCKED] |
| 13 | TMC5160 | [SPEC] |
| 14 | ESP32-S3 + esp-hal no_std/embassy | [LOCKED] |
| 15 | Pulldown invariant between modes; mode via switch | [LOCKED] |
| 16 | Leader-marking two-track procedure | [SPEC] |
| 17 | UI layout §9.3 | [SPEC — review] |
