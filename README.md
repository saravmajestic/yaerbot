# yaerbot — Farm OS seeding robot

An autonomous seeding robot for the **Arduino Physical AI Challenge India 2026**, built on
the **Arduino UNO Q**. It acts as an end-to-end agronomic agent: it _plans_ (what/when to
seed), _acts_ (seeds a plot — plain fixed-spacing or by following a drip line), and _reports_
(a pictorial map of what it planted).

> **yaer** — from **ஏர்** (_ēr_), the Tamil word for the plough: the tool that opened the
> soil before sowing.

## Why

Smallholders sow by hand, broadcasting the seed. That costs three ways:

- **Seed** — it scatters unevenly, so you over-sow to make up for the gaps.
- **Weeding** — scattered seed leaves no clean rows to weed between, so weeding stays slow
  hand work and takes longer than the sowing did.
- **Water** — on drip-irrigated land the water only reaches the emitters. Seed that lands
  anywhere else is barely watered.

This robot places one seed at a time at the spacing you set. On drip land it **finds the
emitters by sight** and seeds to them, so every seed sits where the water already goes. It
picks the date from the local panchangam and biodynamic calendar — groundnut goes in on
**Keel Nokku Naal** — and hands back a map of what it planted.

## Architecture

```mermaid
flowchart TB
  CAM["USB camera<br/><i>320x240 at 30 fps</i>"]
  BROW["Operator's browser<br/><i>phone or laptop</i>"]
  PWR["3S LiPo &rarr; 20 A main &rarr; branch fuses &rarr; 12 V to 5 V buck"]

  subgraph BOARD["Arduino UNO Q &mdash; one board, nothing else"]
    direction TB
    LIN["<b>Qualcomm QRB2210 &middot; Linux</b> &mdash; thinking, no deadlines<br/>web console &middot; tube detection (OpenCV)<br/><b>emitter model (FOMO)</b> &middot; <b>crop planner (LLM)</b><br/>path planner &middot; run log &middot; farm-map report"]
    BR{{"RouterBridge &mdash; 15 RPCs down, getDiag back"}}
    MCU["<b>STM32U585 &middot; microcontroller</b> &mdash; acting, to the millisecond<br/>motor PWM &middot; two servos<br/>solenoid punch (500 ms, atomic) &middot; gyro I&sup2;C"]
    LIN --- BR --- MCU
  end

  DRV["2x IBT-2 &rarr; 4 gear motors"]
  SEED["Seeder &mdash; spool &middot; drum &middot; punch"]
  IMU["MPU-6050 gyro <i>(Wire2, A4/A5)</i>"]

  BROW <-->|"WiFi &mdash; the robot serves its own hotspot,<br/>or joins a network you configure (one radio, so one or the other)"| LIN
  CAM -->|USB| LIN
  PWR --> BOARD
  MCU --> DRV
  MCU --> SEED
  IMU --> MCU

  classDef ai fill:#EAF5EE,stroke:#2E8B57,stroke-width:2px,color:#1F2A36
  classDef per fill:#F7F0DC,stroke:#A6844A,color:#1F2A36
  classDef pwr fill:#FBEDEB,stroke:#C0392B,color:#1F2A36
  classDef brg fill:#EAF1F9,stroke:#2E6FB0,stroke-width:2px,color:#1F2A36
  classDef fast fill:#FBF4EC,stroke:#A6844A,color:#1F2A36
  class LIN ai
  class MCU fast
  class BR brg
  class CAM,IMU,DRV,SEED,BROW per
  class PWR pwr
```

A fuller version, laid out around the same idea and sized for a slide or a video frame:
[`docs/architecture.svg`](docs/architecture.svg) (PNG: [`architecture@2x.png`](docs/architecture@2x.png)).
Circuit-level detail is in [`docs/schematic/`](docs/schematic/index.html).

- **Arduino UNO Q** — Qualcomm Linux side (App Lab Python app, on-device CV + LLM) **+**
  STM32U585 MCU (real-time GPIO / motor + seeder control over RouterBridge RPCs).
- **Camera** — a **USB webcam** on the board, 320×240 at a steady 30 fps. It replaced an
  ESP32-CAM that managed only 0.7–6.7 fps over WiFi; the firmware and the measurements behind
  the switch are kept in [`firmware/esp32_cam/`](firmware/esp32_cam/) and
  [`docs/usb-camera.md`](docs/usb-camera.md).
- **On-device AI** — an Edge Impulse **FOMO** model for emitter detection, and a small LLM via
  Ollama for the planner, run **staggered, never concurrent** with the vision model.
- **Sensing** — the camera and the gyro. No soil probes and no GPS are fitted.

## How the UNO Q is used

Both halves of the board do real work, and the split between them is what makes the seeder
accurate.

**The seeder's punch must hold for exactly 500 ms while a neural network runs on the same
board.** On a single-processor system those two compete. Here they don't: `plantSeed` is a
single RPC that the **STM32 executes atomically** — drum release, 100 ms settle, coil on,
500 ms, coil off — so a busy Linux side, a garbage-collection pause, or even a dropped
bridge cannot stretch the time a pulse-rated solenoid is energised. Nothing on the Qualcomm
side has to meet a deadline, because none of the timing lives there.

| UNO Q capability                         | What depends on it                                                                                                                                                                                                      |
| ---------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **STM32U585 MCU** — real-time GPIO       | motor PWM, both servos, the solenoid's timed punch, the gyro on `Wire2`. 15 RPCs: `setMotors` `stop` `plantSeed` `indexSpool` `dropSeed` `punch` `retract` `getMoisture` `getTemperature` `getEC` `getBattery` `getDiag` … |
| **Qualcomm Linux side** — compute        | the web console, path planning, the LLM planner, OpenCV, and Edge Impulse inference                                                                                                                                     |
| **RouterBridge** — the link between them | every actuator command, and `getDiag`, which reports back from the MCU what actually reached the driver pins                                                                                                            |
| **App Lab bricks**                       | `arduino:web_ui` serves the operator console; `arduino:object_detection` runs the trained model in its own container                                                                                                    |
| **One board, nothing else**              | the UNO Q runs the LLM, the CV and the web console itself — no companion computer                                                                                                                                        |

### Two AI workloads, on the board, no network

- **A small LLM via Ollama** turns the computed sowing recommendation into plain language
  (`qwen2.5:1.5b` by default; `gemma2:2b` is installed and selectable)
- **An Edge Impulse FOMO model** finds drip emitters in the camera feed

Both run **on the board**; nothing calls an API. They are run **staggered, never
concurrently** — at ~1 GB free with a model resident, and vision needing the CPU during a
run, sharing them would starve both. The planner finishes before the robot moves.

The vision model was also **trained on data this robot collected itself**: the Camera tab
drives the drip line and saves frames, those frames are labelled and trained into a FOMO
model, and the model is installed back onto the same board. The loop closes on one device.

### Fully committed I/O

The pin budget is spent, and it drove real design decisions. The seeder's servos had to move
to **digital** pins because `Servo` gives no motion at all on A3 — which then freed A3 for the
solenoid's MOSFET gate. And **A4/A5 went to the gyro**, because they carry `Wire2` (i2c3), the
only I2C the board actually exposes: the header pins silk-screened SDA/SCL have no I2C
peripheral behind them on this core. That cost the battery monitor its input.

Full sheets, colour-coded by net: [`docs/schematic/`](docs/schematic/index.html).

### Board behaviour we had to find the hard way

Documented in [`troubleshooting.md`](docs/troubleshooting.md) in case they save someone else time:

- `pinMode()` on a PWM pin **before** `analogWrite()` kills PWM on that pin (outputs ~0 V)
- `Servo` works only on **digital** pins; A3 gives no motion at all
- ADC full scale is **VREF+ ≈ 3.3 V**, not 5 V
- `Servo.detach()`/`attach()` at runtime **hangs the MCU**, taking the bridge with it
- `halt`/`poweroff` auto-restart while power is applied — only cutting power keeps it off

## Status

**All four acts run end-to-end on the real robot.** Plain-land seeding does multi-row
serpentine on soil with a real row change, logged positions and a generated farm map. Drip
seeding follows the lateral, stops at detected emitters and plants within ±5 cm (measured +0
to +10 mm). Turns are closed-loop on the gyro — worst error 0.3° across four consecutive 90°
turns. The planner runs on-device with no network call.

Two vision limits are open and worth stating plainly: the emitter model also fires on plain
tube, so a 12-emitter row draws about 14 stops; and a hard shadow across the row stops the
tube detector, which waits rather than steering blind. Both are written up with their
evidence — see the demo plan.

## Repo layout

```
firmware/                C++ for the two microcontrollers
  farm_os/               STM32U585 sketch — motor/seeder/sensor RPCs over RouterBridge
  esp32_cam/             ESP32-CAM — superseded by the USB webcam; kept for the record

console/                 the App Lab application that runs ON the robot
  app.yaml               declares the bricks (web_ui, object_detection)
  python/main.py         backend: web UI, Bridge RPCs, camera loop, run state machine
  python/vision/         on-device CV
    vision.py              detect_tube — 4-band brightness profile + robust line fit;
                           detect_crossing — the next lateral, for the traverse
    emitter_ml.py          Edge Impulse FOMO detector, classical fallback
    camera.py              frame source: USB camera by sysfs name, MJPEG stream, or file
  assets/                web operator console — index.html / app.js / style.css

farmos/                  hardware-free brain, testable off the robot
  config.py                SeedPlan — the run config
  path.py                  boustrophedon path planner (Act 2)
  robot_io.py              RobotIO: SimRobot (sim) + BridgeRobot (real UNO Q)
  executor.py              timed dead-reckoning executor + RunLog
  report.py                RunLog -> SVG farm-map report (Act 4)
  planner/                 Act 1 — on-device LLM seeding advisor

scripts/                 field_test.py — field calibration + run harness (solve/tsolve,
                         row, cycle, spool); gen_schematic.py / gen_arch.py — the diagrams;
                         lock_focus.py — find and pin the camera's focus;
                         score_emitter_model.py — grade a retrain against saved frames
tests/                   214 tests — unit gates plus hand-labelled field frames, so a
                         detector that finds nothing fails. Run: python -m pytest
examples/                offline demos — the four acts end to end, no hardware
runs/                    REAL artifacts off the robot: run logs, generated report,
                         MCU per-move diagnostics, battery telemetry
webchat/                 standalone planner-chat prototype (the console has its own Plan tab)
deploy/                  systemd units + install script
docs/                    see the index below
```

## Documentation

| Doc                                               | What it covers                                                                                                                                                                                                                       |
| ------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| [`demo-plan.md`](docs/demo-plan.md)               | the 4-act storyboard, locked decisions, build status                                                                                                                                                                                 |
| [`bom.md`](docs/bom.md)                           | **bill of materials, as built** — plus what was evaluated and rejected                                                                                                                                                               |
| [`schematic/`](docs/schematic/index.html)         | **circuit schematics, as built** — power distribution and protection, drive, seeder, gyro. Colour-coded by net; generated from code by [`scripts/gen_schematic.py`](scripts/gen_schematic.py) so they cannot drift from the firmware |
| [`seeder.md`](docs/seeder.md)                     | the seeding mechanism — spool, metering drum, solenoid punch                                                                                                                                                                         |
| [`ml-emitter-model.md`](docs/ml-emitter-model.md) | **the AI workflow** — capture → label → train FOMO → deploy on-device                                                                                                                                                                |
| [`drive-precision.md`](docs/drive-precision.md)   | open-loop drive model, calibration method, error budget                                                                                                                                                                              |
| [`troubleshooting.md`](docs/troubleshooting.md)   | every hardware fault hit during the build, with root cause and fix                                                                                                                                                                   |
| [`runs/`](runs/)                                  | **real data off the robot** — run logs, the generated farm map, MCU diagnostics, battery telemetry                                                                                                                                   |
| [`usb-camera.md`](docs/usb-camera.md)             | **why the camera changed** — the frame-rate measurements that retired the ESP32-CAM, and the focus lock                                                                                                                               |
| [`architecture.svg`](docs/architecture.svg)       | the system diagram at slide and video size                                                                                                                                                                                           |
