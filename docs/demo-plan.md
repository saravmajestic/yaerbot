# Farm OS — Recorded Demo Plan & Script

### Arduino Physical AI Challenge India 2026

**Deliverable:** a *recorded* demo video **+ submitted source code** (code is reviewed).
**Two rules that drive every decision:**
1. **We control the environment** → real but *prepped* land, chosen plot, flat light, multiple takes, pre-warm/pre-cache anything slow.
2. **The code is read** → everything is genuinely functional and **config/data-driven** (real algorithms, real logged data, generated report). Staging the scene is fair; faking results in code is not.

> This doc is the demo storyboard + tracked checklists — the source of truth for the
> recorded demo and what's built vs. pending. Code lands here module-by-module as each act
> is assembled.

---

## Locked decisions (from design finalization)

| Topic | Decision |
|---|---|
| Surface | **Real, prepped land** (weed-cleared, ~level), recorded outdoors in flat light |
| Drip detection | **On-device trained model (Edge Impulse FOMO) PRIMARY** + classical CV fallback — this is the on-device "Physical AI" claim |
| Vision toolchain | Existing **capture flow → Edge Impulse FOMO → deploy via App Lab `object_detection` brick** (confirmed by `emitter_ml.py`) |
| Planner brain | **Gemma 2B via Ollama** (staggered with vision, never concurrent). Reconciliation is deterministic; the LLM only **presents** the computed data (`data-in→prose-out`) — agentic tool-calling at ≤2B proved unreliable. Qwen 1.5B = fallback. Exact tag pending on-board `free -m`/latency |
| Planner data | Two **real** calendars cached (Tamil panchangam Nokku Naal + biodynamic root/leaf/flower/fruit) + **real** weather (Open-Meteo). Prices are **MOCK for now** (real Agmarknet stubbed). Reconciliation computes dates; the model never invents them |
| Plain-field spacing | **Timed dead-reckoning** (no new hardware, no pins), calibrated on the plot; report logs the executed path. Trailing-wheel encoder = optional stretch |
| Drip spacing | **Model-driven** — plant at each detected emitter (no odometry needed) |
| Report | **Generated from logged positions** (plain: commanded path; drip: emitter detections) |
| **De-scoped for this demo** | **GPS** (timed DR instead), **soil mapping**, **voice/TTS** — all cut from the recorded story (keep as post-demo/stretch) |

---

## The Script — 4 acts

### Act 1 — Planner (software) 🎬
Farmer opens the **Chat tab**, says "I want to plant groundnut." Robot's on-device LLM:
analyzes crop + climate, checks **local biodynamic calendar** (e.g. *Keel Nokku Naal*),
pulls market/weather context, and **recommends a seeding date + spacing config**, explaining
why. *Recording note: data pre-cached; model pre-warmed; the code still makes the real calls.*

### Act 2 — Plain-land seeder (hardware + software) 🎬
Date confirmed → farmer marks a plot (e.g. 4×8 ft, or enter dims). Robot **computes a
boustrophedon path** for the chosen spacing and executes it with **timed dead-reckoning**,
planting at fixed spacing, **no arm rotation**. *Show the computed path overlay on screen.*

### Act 3 — Drip-line seeder (hardware + software) 🎬
Plot has drip → robot uses its **custom on-device model** to find the drip line + emitters,
**follows the lateral**, and plants **at each emitter — no rotation first** (reliable), then
"if we're adventurous" the **rotation-per-emitter** finale. *Show the live CV overlay (tube +
emitter boxes) as proof the model is running on the board.*

### Act 4 — Report (software) 🎬
Seeding done → robot **generates a pictorial farm map**: plot outline, seed dots at recorded
positions, spacing annotations, seed count, and the Act-1 rationale — closing the loop.

---

## Track A — Hardware Demo (physical seeding)

Status: ✅ done · 🟡 partial (bits exist, not demo-ready) · ⬜ not started

- [✅] **4WD drive** over RouterBridge — all wheels, both directions, survives power-cycle *(fixed the UNO Q `pinMode`-kills-PWM bug this session; persists via `arduino-flash.sh`)*
- [✅] **Seeder mechanism bench-verified** — S3003 spool, SG90 drum (drop 165°), solenoid punch (fires @12V)
- [✅] **Seeder firmware RPCs** — `indexSpool/dropSeed/punch/retract/plantSeed`
- [✅] **Servo + motor-PWM coexistence** confirmed on-device (both drive *and* seeder servos work together)
- [🟡] **Robust field connectors** — IBT-2 signal header loosens under vibration; direct-solder + strain-relief plan agreed, in progress
- [⬜] **Seeder mounted on chassis**, arm/tip reaches soil at demo ride height
- [⬜] **Act 2: plain-land fixed-spacing seeding** run end-to-end on land (path executed + seeds placed)
- [⬜] **Act 3a: drip-follow seeding, no rotation** run end-to-end on the real drip line
- [⬜] **Act 3b: rotation-per-emitter seeding** (`indexSpool`→`plantSeed`) — currently a UI "soon" stub
- [⬜] **Untethered power run** for a full take (LiPo → fuse → switch → buck/IBT/solenoid), stable under motion

---

## Track B — Software Demo (planner, path, model, report)

### B1 — Planner (Act 1)
- [✅] **Data pipeline (calendars)**: Tamil panchangam (nakshatra→Nokku Naal) + biodynamic root/leaf/flower/fruit — real data cached to JSON (`farmos/planner/data/`)
- [✅] **Reconciliation**: `planner.survey()` returns the full picture (both-systems recommendation + panchangam-only / biodynamic-only alternatives + kari-naal avoid days); `recommend()` picks the dual-validated date. groundnut→2026-09-03. Tested.
- [✅] **LLM presentation** (`llm.present()`, data-in→prose-out): the LLM presents the computed data + alternatives; dates come from the reconciliation, not the model. **Gemma 2B > Qwen 1.5B** at faithful presentation (Qwen fudged alternatives; agentic tool-calling at ≤2B is unreliable — kept `converse()` but not the demo path).
- [🟡] **Prices**: MOCK 3-yr monthly history (groundnut/corn/sesame, `prices_mock.json`) wired into the presentation; real Agmarknet (data.gov.in) path stubbed for a one-line swap
- [✅] **Weather**: REAL — live Open-Meteo (no API key) with cache-fallback; near-term outlook + honest horizon flag (climatological normals for a date >16 days out = follow-up). Farm = Salem, TN
- [🟡] **Chat/Plan tab**: standalone prototype built (`webchat/` — chat UI + server calling the planner + Gemma, dark console theme); integrate into the operator console as a tab next (later: voice)
- [✅] **On the UNO Q**: Ollama + Gemma/Qwen installed under `/home` (root untouched, 12 GB free); chat e2e works on-device. **Speed: Gemma ~1.9 tok/s, Qwen ~2.7 tok/s → a full answer ~1–2 min on this CPU** (RAM fine, ~1 GB free with model resident). For the demo: show the instant deterministic card, pre-warm the model, stream a short prose summary — the LLM is a 'reasoning step,' not snappy chat.

### B2 — Path planning + spacing (Act 2)
- [🟡] **Timed space-based seeding mode** exists in the Seed panel (gap × speed, single-spot) — *superseded by the path executor below*
- [✅] **Boustrophedon planner** — `farmos/path.py`: plot dims + spacing → serpentine seed waypoints (computed, axis-aligned; unit-tested)
- [✅] **Timed dead-reckoning executor** — `farmos/executor.py` + `BridgeRobot`. **One seeding row verified on hardware 2026-08-11: three even 40 cm hops + a real 90° pivot.** The same run days earlier "rotated randomly within a 30 cm square". What it took:
  - **Dead-time model on forward** (`startup_s + d/speed`) — a blind 40 cm hop was previously issued as a sub-dead-time burst that barely translated.
  - **Coast model on turns** (negative offset) + a **separate, lower turn duty** — coast is ~75° at PWM 180 vs ~38° at PWM 120, so a 90° turn at full duty is mostly coast.
  - **Three hardware faults fixed first**, none of them "the trim": loose IBT-2 signal jumpers (the *same* command drove 2.6 m straight, then a half-circle an hour later), one wheel not touching (a rigid 4-wheel frame rests on 3 of 4 points), and ~5% normal motor variance. Full writeup: **`ai-labs/apps/farm-robot/docs/farm-os/drive-precision.md`**.
- [✅] **Decision: single stop-and-go flow for BOTH acts** — `stop → drive-a-hop → stop → plant → resume`, driven by a timer (Act 2) or the CV emitter event (Act 3). No continuous mode (punch + rotation-seeding need a dead stop).
- [✅] **Calibration (2026-08-11, hard floor, 3S ~12 V)**: `ltrim=0.75`, `speed=0.616`, `startup=0.104`, `tpwm=120`, `tdps=51`, `tstartup=-0.75` → 40 cm hop = 0.75 s, 90° turn = 1.01 s. Recalibrate after **any** mechanical/wiring change (`field_test.py solve`/`tsolve`).
- [✅] **Self-checking field tests** — `getDiag` on the MCU + `BridgeRobot._check_diag`: every move compares what we sent vs what the MCU latched and drove, and prints a `!!` banner on mismatch. Also a **Diag tab** in the console tracing browser → console → MCU → driver pins → motor current.
- [🟡] **Turn precision is the remaining gap** — ±5–10° per turn from coast. The **row change (`A→B→B1→C`) doubles it**: both 90° turns go the same way, so 10°/turn leaves the next row 20° skew. Mitigations added: lower turn duty, optional **deceleration ramp** (`tramp`), and a **`uturn` calibration mode** that tunes the row change as one primitive (measure "are the legs parallel" + lateral gap). Validate on real ground.
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
- [⬜] **Field-tune** classical CV fallback thresholds against real footage

### B4 — Report (Act 4)
- [✅] **Report generator** — `farmos/report.py`: RunLog → self-contained **SVG farm map** (plot, boustrophedon path, planned vs planted dots, spacing/drift stats, planner rationale); unit-tested
- [⬜] Report view/export in the UI

### Supporting (already done)
- [✅] Tabbed operator console (Drive/Seed/Soil/Cam/Settings), deployed to board
- [✅] Battery monitor (10k/2k on A4, `ADC_VREF=3.3`) — live %, survives power-cycle

---

## Build order (recorded-demo optimized)

De-risk by getting one **complete** thread first, then add the ambitious pieces on top:

1. **Track A field-readiness** — connectors, mount seeder, untethered power run *(unblocks every physical take)*
2. **Act 2 end-to-end** — boustrophedon planner → timed DR executor → logged positions → **Act 4 report** *(a complete, low-risk story on its own)*
3. **Act 1 planner** — Ollama + chat tab + cached data pipeline *(high wow, independent of mechanics)*
4. **Act 3a** — collect/label/train/deploy FOMO → drip-follow **no-rotation** seeding on the real line
5. **Act 3b** — rotation-per-emitter finale *(stretch; don't let it own the demo clock)*

**Fallback:** if time runs short, Acts 1 + 2 + 4 already form a full recorded story; Act 3 (the CV flex) layers on top.

---

## Honest status (2026-08-09)

**Foundations solid, nothing demo-complete yet.** Drive, camera pipeline, seeder mechanism +
RPCs, classical CV, capture flow, and battery are ✅ done and verified. But **every one of the
four acts is still ⬜/🟡 as an end-to-end demo**: no planner, no path executor, no trained
model deployed, no report, no on-land seeding run recorded. Next concrete step per build order:
**Track A field-readiness**, then close **Act 2 + Act 4** as the first complete thread.
