# Farm OS — Recorded Demo Plan & Script

### Arduino Physical AI Challenge India 2026

**Deliverable:** a recorded demo video **+ source code**.

## Why this robot

Sowing on a smallholding is decided by two things at once: *when* to sow, which here still
follows the local panchangam and biodynamic calendar (groundnut goes in on **Keel Nokku
Naal**), and *where* each seed lands. Both are done by hand, and both are where waste
happens.

- **Seed** — hand-broadcasting over-sows to cover for uneven placement. Fixed-spacing
  mechanical placement puts one seed where one seed is wanted.
- **Water** — on drip-irrigated land the water arrives at the emitters. A seed placed
  anywhere else is irrigated indirectly or not at all. Seeding *to the emitters* means
  every seed sits where water already goes, which is why the robot finds them by sight
  rather than assuming a spacing.
- **Soil and labour** — a light robot punching a single hole per seed disturbs far less
  soil than a tilled seedbed, and sowing is short, hot, and hard to hire for.

The robot therefore has to do the whole job, not one slice of it: decide the date from
local practice and real weather, place seeds accurately on either kind of land, and hand
the farmer a record of what it did.

**The principle for the build:** everything is genuinely functional and data-driven — real
algorithms, real logged data, a generated report. The numbers in the report come from what
the robot actually did.

---

## Locked decisions (from design finalization)

| Topic | Decision |
|---|---|
| Surface | **Prepared seedbed** — weed-cleared and roughly levelled, i.e. the state land is actually in when it is sown |
| Drip detection | **On-device trained model (Edge Impulse FOMO) PRIMARY** + classical CV fallback — this is the on-device "Physical AI" claim |
| Vision toolchain | Existing **capture flow → Edge Impulse FOMO → deploy via App Lab `object_detection` brick** (confirmed by `emitter_ml.py`) |
| Planner brain | **Gemma 2B via Ollama** (staggered with vision, never concurrent). Reconciliation is deterministic; the LLM only **presents** the computed data (`data-in→prose-out`) — agentic tool-calling at ≤2B proved unreliable. Qwen 1.5B = fallback. Exact tag pending on-board `free -m`/latency |
| Planner data | Two **real** calendars cached (Tamil panchangam Nokku Naal + biodynamic root/leaf/flower/fruit) + **real** weather (Open-Meteo). Prices are **MOCK for now** (real Agmarknet stubbed). Reconciliation computes dates; the model never invents them |
| Plain-field spacing | **Timed dead-reckoning** (no new hardware, no pins), calibrated on the plot; report logs the executed path. Trailing-wheel encoder = optional stretch |
| Drip spacing | **Model-driven** — plant at each detected emitter (no odometry needed) |
| Report | **Generated from logged positions** (plain: commanded path; drip: emitter detections) |
| **Deliberately not used** | **GPS** — consumer accuracy (±2–5 m) is worse than dead reckoning over a 3–6 m plot and gives no heading at rest; the problem is plot-scale *relative* odometry. **Voice/TTS** — the planner is a reasoning step, not a conversation, and prose is read faster than it is spoken. |

---

## The Script — 4 acts

> **Hardware note for each act below** — one line to say on camera. The architecture and
> the reasoning behind it are in the [README](../README.md#how-the-uno-q-is-used).

### Act 1 — Planner (software) 🎬
Farmer opens the **Chat tab**, says "I want to plant groundnut." Robot's on-device LLM:
analyzes crop + climate, checks **local biodynamic calendar** (e.g. *Keel Nokku Naal*),
pulls market/weather context, and **recommends a seeding date + spacing config**, explaining
why.
> 🔧 *"This model is running on the robot. There's no internet call in this answer."*

### Act 2 — Plain-land seeder (hardware + software) 🎬
Date confirmed → farmer marks a plot (e.g. 4×8 ft, or enter dims). Robot **computes a
boustrophedon path** for the chosen spacing and executes it with **timed dead-reckoning**,
planting at fixed spacing, **no arm rotation**. The computed path is shown as an overlay while it runs.
> 🔧 *"The Linux side planned this path; the microcontroller is driving it. Each punch is
> timed by the MCU, so it stays exact no matter what the rest of the board is doing."*
> Cut to the **Diag tab** for a few seconds — it traces one command from the browser
> through the console to the MCU and out to the driver pins.

### Act 3 — Drip-line seeder (hardware + software) 🎬
Plot has drip → robot uses its **custom on-device model** to find the drip line + emitters,
**follows the lateral**, and plants **at each emitter — no rotation first** (reliable), then
"if we're adventurous" the **rotation-per-emitter** finale. The live CV overlay (tube +
emitter boxes) shows the model running on the board.
> 🔧 *"This model was trained on pictures this robot took of this drip line — and it's
> running on the same board that's driving the wheels."*

### Act 4 — Report (software) 🎬
Seeding done → robot **generates a pictorial farm map**: plot outline, seed dots at recorded
positions, spacing annotations, seed count, and the Act-1 rationale — closing the loop.
> 🔧 *"Every dot is a position the robot logged as it planted it."* Real output of a
> previous run is committed in [`../runs/`](../runs/).

---

## Track A — Hardware Demo (physical seeding)

Status: ✅ done · 🟡 partial (bits exist, not demo-ready) · ⬜ not started

- [✅] **4WD drive** over RouterBridge — all wheels, both directions, survives power-cycle *(fixed the UNO Q `pinMode`-kills-PWM bug this session; persists via `arduino-flash.sh`)*
- [✅] **Seeder mechanism working** — S3003 spool, SG90 drum, solenoid punch. Plant sequence is **seed-first**: the drum releases, then the tip drives the seed in (coil-on 500 ms). The **4-seed cross** (arm 0° then 90° → seeds at 0/90/180/270) runs end-to-end.
- [✅] **Seeder firmware RPCs** — `indexSpool/dropSeed/punch/retract/plantSeed`
- [✅] **Servo + motor-PWM coexistence** confirmed on-device (both drive *and* seeder servos work together)
- [🟡] **Robust field connectors** — the recurring fault class. Power branches rewired in 1 mm² stranded and soldered; signal leads still being audited. Every branch now needs its own fuse sized to its own wire (see `troubleshooting.md` → `[Power]`).
- [⬜] **Seeder mounted on chassis**, arm/tip reaches soil at demo ride height
- [✅] **Act 2: plain-land seeding on soil** — multi-row serpentine (hop → 4-seed cross → row change) executed on real ground with seeds placed; run logged.
- [⬜] **Act 3a: drip-follow seeding, no rotation** run end-to-end on the real drip line
- [✅] **Act 3b: rotation-per-emitter seeding** — `indexSpool`→`plantSeed` per arm position; `indexSpool` now takes a **physical** angle (calibrated: physical 90° ← servo cmd 64), so the cross is a true cross.
- [🟡] **Untethered power run** — runs on LiPo throughout. Outstanding: **branch fuses** (2 A solenoid, 3 A buck) before any further field work.

---

## Track B — Software Demo (planner, path, model, report)

### B1 — Planner (Act 1)
- [✅] **Data pipeline (calendars)**: Tamil panchangam (nakshatra→Nokku Naal) + biodynamic root/leaf/flower/fruit — real data cached to JSON (`farmos/planner/data/`)
- [✅] **Reconciliation**: `planner.survey()` returns the full picture (both-systems recommendation + panchangam-only / biodynamic-only alternatives + kari-naal avoid days); `recommend()` picks the dual-validated date. groundnut→2026-09-03. Tested.
- [✅] **LLM presentation** (`llm.present()`, data-in→prose-out): the LLM presents the computed data + alternatives; dates come from the reconciliation, not the model. **Gemma 2B > Qwen 1.5B** at faithful presentation (Qwen fudged alternatives; agentic tool-calling at ≤2B is unreliable — kept `converse()` but not the demo path).
- [🟡] **Prices**: MOCK 3-yr monthly history (groundnut/corn/sesame, `prices_mock.json`) wired into the presentation; real Agmarknet (data.gov.in) path stubbed for a one-line swap
- [✅] **Weather**: REAL — live Open-Meteo (no API key) with cache-fallback; near-term outlook + honest horizon flag (climatological normals for a date >16 days out = follow-up). Farm = Salem, TN
- [🟡] **Chat/Plan tab**: standalone prototype built (`webchat/` — chat UI + server calling the planner + Gemma, dark console theme); integrate into the operator console as a tab next (later: voice)
- [✅] **On the UNO Q**: Ollama + Gemma/Qwen installed under `/home` (root untouched, 12 GB free); chat e2e works on-device. **Speed: Gemma ~1.9 tok/s, Qwen ~2.7 tok/s → a full answer ~1–2 min on this CPU** (RAM fine, ~1 GB free with model resident). **The architecture follows from that:** the deterministic reconciliation renders instantly as a card and the LLM's prose streams in after — the recommendation never waits on the model, because the model does not produce it.

### B2 — Path planning + spacing (Act 2)
- [🟡] **Timed space-based seeding mode** exists in the Seed panel (gap × speed, single-spot) — *superseded by the path executor below*
- [✅] **Boustrophedon planner** — `farmos/path.py`: plot dims + spacing → serpentine seed waypoints (computed, axis-aligned; unit-tested)
- [✅] **Timed dead-reckoning executor** — `farmos/executor.py` + `BridgeRobot`. **One seeding row verified on hardware 2026-08-11: three even 40 cm hops + a real 90° pivot.** The same run days earlier "rotated randomly within a 30 cm square". What it took:
  - **Dead-time model on forward** (`startup_s + d/speed`) — a blind 40 cm hop was previously issued as a sub-dead-time burst that barely translated.
  - **Coast model on turns** (negative offset) + a **separate, lower turn duty** — coast is ~75° at PWM 180 vs ~38° at PWM 120, so a 90° turn at full duty is mostly coast.
  - **Three hardware faults fixed first**, none of them "the trim": loose IBT-2 signal jumpers (the *same* command drove 2.6 m straight, then a half-circle an hour later), one wheel not touching (a rigid 4-wheel frame rests on 3 of 4 points), and ~5% normal motor variance. Full writeup: **`ai-labs/apps/farm-robot/docs/farm-os/drive-precision.md`**.
- [✅] **Decision: single stop-and-go flow for BOTH acts** — `stop → drive-a-hop → stop → plant → resume`, driven by a timer (Act 2) or the CV emitter event (Act 3). No continuous mode (punch + rotation-seeding need a dead stop).
- [✅] **Calibration (2026-08-16, soil, 3S)**: `ltrim=0.83`, `speed=0.628`, `startup=0.099`, `tpwm=120`, `tdps=45.2`, `tstartup=-0.80`. Recalibrate after **any** mechanical/wiring change (`field_test.py solve`/`tsolve`). Three separate trim values had drifted apart across the console, the plot runs and the field tests — all now single-sourced.
- [✅] **Self-checking field tests** — `getDiag` on the MCU + `BridgeRobot._check_diag`: every move compares what we sent vs what the MCU latched and drove, and prints a `!!` banner on mismatch. Also a **Diag tab** in the console tracing browser → console → MCU → driver pins → motor current.
- [🟡] **Turn precision is the remaining gap** — ±5–10° per turn from coast, and it moves with the surface: a skid-steer pivot rotates by scrubbing the wheels sideways, so hard floor and loose tilth genuinely differ. Arc (radial) steering was added to the drive controls for gentle correction, which scrubs far less than a pivot. The **row change (`A→B→B1→C`) doubles it**: both 90° turns go the same way, so 10°/turn leaves the next row 20° skew. Mitigations added: lower turn duty, optional **deceleration ramp** (`tramp`), and a **`uturn` calibration mode** that tunes the row change as one primitive (measure "are the legs parallel" + lateral gap). Validate on real ground.
- [⬜] **MPU6050 gyro (~₹150)** — closed-loop heading. The only real fix for turn accuracy (measures the angle instead of predicting it) and it **removes the need for trim** entirely. Free `SDA`/`SCL` pins. Strongly reconsider before the final recording.
- [⬜] **Wheel encoder** (LM393 slot sensor, ~₹100) — closed-loop *distance*. Note encoders do **not** fix heading in a skid-steer (wheels slip by design during a turn); that's the gyro's job.

### B3 — Vision / on-device model (Act 3)
- [✅] Classical CV — `detect_tube` (Canny→Hough→steering) + `detect_emitter` (offline-tested 4/4, 3/3)
- [✅] `camera.py` source abstraction; `tube_follow.py` steer→plant loop *(on-device, end-to-end untested)*
- [✅] **Camera tab** — UNO Q sole stream consumer, pushes annotated frames; **Drive-tab live feed fixed** (this session) + auto-connect
- [✅] **ESP32-CAM** flashed/streaming (mDNS `farmcam.local`), OpenCV present in container
- [✅] **`emitter_ml.py` FOMO scaffold** + ML-first-with-classical-fallback wired into `_cam_loop`
- [✅] **Dataset-capture tool** (Camera tab: start/stop, interval, thumbnails, saves raw frames to `~/captures`)
- [⬜] **Collect** field footage of the actual drip tube + emitters (cam mounted, demo lighting) — ~100–300 imgs
- [⬜] **Label + train FOMO** in Edge Impulse (center-dot labels; class name must match `EMITTER_LABELS`)
- [⬜] **Deploy** to UNO Q; verify `detect()` shape vs `_extract_boxes()`; `ml_available()` true
- [✅] **Dataset collected on the real drip line** (scan mode drives the tube and captures; the seeder is never touched)
- [✅] **FOMO model trained + verified on the board** — 160×160, class `emitter`; detects at **99.3% confidence** on a real capture, run directly against the model runner
- [🟡] **Deploy to the brick** — the model is on the board and the runner container is healthy; installing it through App Lab needs the USB cable
- [⬜] **Field-tune** classical CV fallback thresholds against real footage

### B4 — Report (Act 4)
- [✅] **Report generator** — `farmos/report.py`: RunLog → self-contained **SVG farm map** (plot, boustrophedon path, planned vs planted dots, spacing/drift stats, planner rationale); unit-tested
- [⬜] Report view/export in the UI

### Supporting (already done)
- [✅] Tabbed operator console (Drive/Seed/Soil/Cam/Settings), deployed to board
- [✅] Battery monitor (10k/2k on A4, `ADC_VREF=3.3`) — live %, survives power-cycle

---

## Build order

One **complete** thread first, then the more ambitious pieces on top:

1. **Track A field-readiness** — connectors, mount seeder, untethered power run *(unblocks every physical take)*
2. **Act 2 end-to-end** — boustrophedon planner → timed DR executor → logged positions → **Act 4 report** *(a complete, low-risk story on its own)*
3. **Act 1 planner** — Ollama + chat tab + cached data pipeline *(high wow, independent of mechanics)*
4. **Act 3a** — collect/label/train/deploy FOMO → drip-follow **no-rotation** seeding on the real line
5. **Act 3b** — rotation-per-emitter: a 4-seed cross around each emitter

---

## Status (2026-08-16)

**Three of the four acts run end-to-end; the fourth is one cable away.**

| Act | State |
|---|---|
| **1 — Planner** | Reconciliation + real calendars + live weather working on-device; Gemma presents the computed result. Prices are still mock. Chat UI is a standalone prototype, not yet a console tab. |
| **2 — Plain-land seeder** | ✅ **Runs on soil.** Multi-row serpentine with a real row change, 4 seeds per stop, run logged and reported. |
| **3 — Drip seeder** | Tube-following works on the real line. The trained model detects emitters at 99.3% but is not yet installed through App Lab (needs the USB cable), so the classical detector is what currently drives it. |
| **4 — Report** | ✅ Generated from logged positions — see [`../runs/`](../runs/) for real output. |

**The honest weak point is reliability, not capability.** Every subsystem works; what still
bites is the physical build — five separate connection faults in one week, one of which set
a wire alight. The fixes are known and unglamorous: branch fuses sized to their own wire,
solder instead of Dupont on anything carrying current, and flashing the camera firmware so a
dropped WiFi association recovers by itself instead of needing a power cycle.

**Next, in order:** branch fuses → camera firmware → repeat full runs and count how many
succeed. Nothing on that list is a new feature.
