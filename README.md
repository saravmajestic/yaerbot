# yaerbot — Farm OS seeding robot

An autonomous seeding robot for the **Arduino Physical AI Challenge India 2026**, built on
the **Arduino UNO Q**. It acts as an end-to-end agronomic agent: it *plans* (what/when to
seed), *acts* (seeds a plot — plain fixed-spacing or by following a drip line), and *reports*
(a pictorial map of what it planted).

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
Foundations working (drive, camera pipeline, seeder mechanism + firmware RPCs, classical CV,
battery). The four demo acts are still being assembled end-to-end — see the status table in
the demo plan. Code is copied in module-by-module as each act is built.

## Repo layout (populated as the demo is built)
```
docs/         project docs — start with demo-plan.md
```
