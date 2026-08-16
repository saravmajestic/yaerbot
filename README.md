# yaerbot — Farm OS seeding robot

An autonomous seeding robot for the **Arduino Physical AI Challenge India 2026**, built on
the **Arduino UNO Q**. It acts as an end-to-end agronomic agent: it *plans* (what/when to
seed), *acts* (seeds a plot — plain fixed-spacing or by following a drip line), and *reports*
(a pictorial map of what it planted).

## Why

Sowing on a smallholding turns on two questions, and both are answered by hand. *When* —
which here still follows the local panchangam and biodynamic calendar; groundnut goes in
on **Keel Nokku Naal**. And *where* each seed lands.

Hand-broadcasting over-sows to compensate for uneven placement, so seed is wasted by
design. On drip-irrigated land the water arrives at the emitters, so a seed placed
anywhere else is watered indirectly or not at all — which is why this robot **finds the
emitters by sight** and seeds to them, rather than assuming a spacing. And punching one
hole per seed disturbs far less soil than preparing a tilled seedbed.

So the robot has to do the whole job rather than one slice of it: decide the date from
local practice and real weather, place seeds accurately on either kind of land, and hand
back a record of what it did.

## The demo (4 acts)
1. **Planner** — an on-device LLM recommends a crop's seeding date + spacing from local
   climate, market context, and the local **biodynamic calendar** (e.g. *Keel Nokku Naal*).
2. **Plain-land seeder** — computes a boustrophedon path over a marked plot and seeds at
   fixed spacing (fixed arm, no rotation).
3. **Drip-line seeder** — a **custom on-device vision model** finds the drip line + emitters,
   follows the lateral, and plants at each emitter (rotation-per-emitter as the stretch).
4. **Report** — generates a pictorial farm map of the seeded positions with spacing.

See **[`docs/demo-plan.md`](docs/demo-plan.md)** for the full storyboard, locked technical
decisions, and build status.

## Architecture
- **Arduino UNO Q** — Qualcomm Linux side (App Lab Python app, on-device CV + LLM) **+**
  STM32U585 MCU (real-time GPIO / motor + seeder control over RouterBridge RPCs).
- **ESP32-CAM** — streams MJPEG; the UNO Q is the sole consumer and runs detection on-device.
- **On-device AI** — Edge Impulse **FOMO** model for drip/emitter detection (classical CV
  fallback); a small **Qwen** LLM via Ollama for the planner (run staggered, never concurrent
  with the vision model).

## Status
Acts 2 and 4 run end-to-end on soil — multi-row serpentine seeding with a real row change,
logged positions, and a generated farm map. The drip model is trained and verified on the
board; the planner runs on-device. Full status table in the demo plan.

## Repo layout
```
firmware/                C++ for the two microcontrollers
  farm_os/               STM32U585 sketch — motor/seeder/sensor RPCs over RouterBridge
  esp32_cam/             ESP32-CAM — MJPEG stream, WiFi tending, /status diagnostics

console/                 the App Lab application that runs ON the robot
  app.yaml               declares the bricks (web_ui, object_detection)
  python/main.py         backend: web UI, Bridge RPCs, camera loop, run state machine
  python/vision/         on-device CV
    vision.py              detect_tube (Canny+Hough steering) + classical detect_emitter
    emitter_ml.py          Edge Impulse FOMO detector, classical fallback
    camera.py              frame source abstraction
  assets/                web operator console — index.html / app.js / style.css

farmos/                  hardware-free brain, testable off the robot
  config.py                SeedPlan — the run config
  path.py                  boustrophedon path planner (Act 2)
  robot_io.py              RobotIO: SimRobot (sim) + BridgeRobot (real UNO Q)
  executor.py              timed dead-reckoning executor + RunLog
  report.py                RunLog -> SVG farm-map report (Act 4)
  planner/                 Act 1 — on-device LLM seeding advisor

scripts/field_test.py    field calibration + run harness (solve/tsolve, row, cycle, spool)
tests/                   58 unit tests — run: python -m pytest
examples/                offline demos — run the whole pipeline with no hardware
runs/                    REAL artifacts off the robot: run logs, generated report,
                         MCU per-move diagnostics, battery telemetry
deploy/                  systemd units + install script
docs/                    see the index below
```

## Documentation
| Doc | What it covers |
|---|---|
| [`demo-plan.md`](docs/demo-plan.md) | the 4-act storyboard, locked decisions, build status |
| [`bom.md`](docs/bom.md) | **bill of materials, as built** — plus what was evaluated and rejected |
| [`uno-q-wiring.md`](docs/uno-q-wiring.md) | **circuit reference** — pinout, inter-box cable, driver wiring, power protection |
| [`seeder.md`](docs/seeder.md) | the seeding mechanism — spool, metering drum, solenoid punch |
| [`ml-emitter-model.md`](docs/ml-emitter-model.md) | **the AI workflow** — capture → label → train FOMO → deploy on-device |
| [`drive-precision.md`](docs/drive-precision.md) | open-loop drive model, calibration method, error budget |
| [`troubleshooting.md`](docs/troubleshooting.md) | every hardware fault hit during the build, with root cause and fix |
| [`runs/`](runs/) | **real data off the robot** — run logs, the generated farm map, MCU diagnostics, battery telemetry |

## Try it (no hardware)
```
python examples/plain_field_demo.py   # writes examples/report.svg + runlog.json
python tests/test_path.py && python tests/test_executor.py && python tests/test_report.py
```
