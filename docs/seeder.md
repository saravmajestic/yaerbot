# Drip-Aligned Adaptive Seeder
### Detailed Architecture & Problem Design

---

## Concept

Standard seeders plant at fixed row intervals — ignoring where water actually reaches. This seeder inverts that: **water location defines planting location.**

Every seed is placed at or around a drip emitter center. Every seed is guaranteed moisture from day one. Zero placement wastage.

Built specifically for drip-irrigated small farms, where emitter spacing defines the effective watering zone per plant.

---

## The Three Core Problems

```
Problem 1: Find emitter centers accurately in real-time
Problem 2: Plant a configurable radial pattern around each center
Problem 3: Handle different crop varieties without hardware changes
```

---

## Problem 1: Emitter Detection

### How Drip Irrigation Looks on the Ground

```
Soil surface:

━━━●━━━━━━━━━━━●━━━━━━━━━━━●━━━━━━━━━━━●━━━  ← drip tube (black LDPE, 12-16mm)
   ▲            ▲            ▲            ▲
emitter      emitter      emitter      emitter
(20–60cm spacing depending on crop)

When drip runs: dark wet circle ~15–20cm diameter around each emitter
```

### Detection Method A — Tube Following (Camera)

Black LDPE tube against brown soil = strong visual contrast.

**Implementation:**
- USB camera mounted at 30cm height on robot front
- OpenCV HSV thresholding isolates dark tube from soil background
- Hough line transform detects tube axis
- Robot steers to keep tube centered in camera frame
- Speed: 0.15 m/s during tube following

**Edge case handling:**
- Soil too dark (wet after rain): switch to near-IR channel or use Method B as primary
- Tube partially buried: follow visible segments, use emitter spacing estimate to bridge gaps
- Row end detection: moisture drops to ambient + no tube visible → turn to next row

---

### Detection Method B — Moisture Peak Detection (Soil Sensor)

Soil moisture reading rises as robot approaches an active emitter and peaks directly above it.

**Implementation:**
- Capacitive soil moisture sensor mounted at front-bottom of robot at emitter height
- Reads continuously at 10Hz while robot moves along tube
- Rolling max-tracking algorithm: when reading drops after a peak → emitter center was at peak position
- Robot backtracks by measured distance to re-center on peak

```
Moisture reading profile as robot moves along tube:

  High |          ╭──╮              ╭──╮
       |       ╭──╯  ╰──╮        ╭──╯  ╰──╮
  Low  |───────╯         ╰────────╯         ╰────
                 ▲                  ▲
            emitter 1           emitter 2
```

**Advantage:** Works even when tube is buried, in low light, or when soil color makes visual detection unreliable.

---

### Detection Method C — Emitter Bump Confirmation (Camera)

Inline drip emitters create a small cylindrical bump on the tube (2–5mm height). Confirmation layer on top of Method A.

**Implementation:**
- Same camera used for tube following
- Secondary CV pass: detect circular protrusion on detected tube line
- Filters out false moisture peaks from uneven soil
- Provides spatial confirmation: *"Bump detected, moisture peaked — high confidence emitter center"*

---

### Fusion Logic (UNO Q — STM32 MCU side)

```
State machine running on STM32 real-time MCU:

FOLLOWING_TUBE
    │
    ├─ moisture reading rising? → enter APPROACHING_EMITTER
    │
APPROACHING_EMITTER
    │
    ├─ moisture peaks AND camera confirms bump? → EMITTER_CONFIRMED (high confidence)
    ├─ moisture peaks, no visual confirm? → EMITTER_LIKELY (medium confidence)
    ├─ visual bump, no moisture peak? → EMITTER_POSSIBLE (low confidence, emitter may be clogged)
    │
EMITTER_CONFIRMED / EMITTER_LIKELY
    │
    └─ STOP → trigger seeder arm → log GPS + readings → resume FOLLOWING_TUBE

EMITTER_POSSIBLE (low confidence)
    │
    └─ SKIP (flag as possible clog) → log GPS → resume FOLLOWING_TUBE
```

**Clogged emitter detection:** If visual bump present but moisture peak absent → emitter is likely clogged. Flagged on farm map. Robot skips seeding here (no water = no point planting).

---

## Problem 2: Radial Pattern Planting

### Seeder Arm Design

Instead of moving the robot body to each seed position (slow, imprecise), a stepper-driven rotating arm executes the radial pattern from a fixed center position.

```
                [Hopper]  ← 3D printed, rear mount
                   │
            [Seed agitator] ← coin vibration motor
                   │
              [Gate SG90]  ← rotates pocket drum, meters 1–2 seeds by volume
                   │
           [silicone tube] ← runs directly outside arm to tip housing
                   │  (hangs freely, flexes at all arm positions)
         ┌────[Stepper hub]────┐
         │   28BYJ-48 stepper  │
         │                     │
    [Arm — solid PETG, 10cm]
         │  solenoid wires taped along outside
         │
  [Tip assembly — at arm end]
    ├─ JF-0530B solenoid (12V, fixed to arm end)
    └─ Hollow nose housing (attached to plunger, moves with it)
         ├─ 15mm hollow bore — passes any seed incl. groundnut/maize
         ├─ side tube socket — silicone tube (14mm) pushes in here
         └─ blunt nose — punches into soil when solenoid fires
```

**Arm materials:**
- Arm body: solid PETG (printed, impact resistant — no hollow channel needed)
- Stepper hub: PETG (printed, Double-D bore grips stepper shaft)
- Tip housing: PETG (printed, hollow nose, moves with solenoid plunger)
- Seed tube: 14mm-ID silicone tubing (routes externally from gate to tip housing socket)
- Solenoid: JF-0530B 12V, controlled via IRLZ44N MOSFET on D11

---

### Execution Sequence Per Emitter

```
T+0s    Robot stops — emitter center aligned under stepper hub

T+1s    Arm at position 0 (0°):
          → vibration motor pulses ~1s (de-bridges seeds into feed throat)
          → gate drum cycles (fill→release): seed drops through hollow bore into hole
          → solenoid fires: housing + cone enters soil (10mm stroke)
          → solenoid releases: housing retracts

T+3s    Arm steps to position 1 (60°) — 683 stepper steps
T+4s      → solenoid fires, gate opens, seed drops, solenoid releases
T+6s    Arm steps to position 2 (120°)
T+7s      → plant
          ... repeat for all configured positions ...
T+12s   Last position planted (max 300° from start — arm never completes full 360°)
T+13s   Arm reverses back to 0° (stepper runs CCW)
          → tube untwists as arm returns (silicone, external routing)
T+15s   UNO Q logs: GPS, timestamp, soil EC, moisture, temp, seeds dropped
T+16s   Robot resumes tube following
```

**Arm sweep rule:** arm moves forward up to 300° max (5 × 60° moves for 6-position pattern), then always reverses to 0°. Never completes a full 360° rotation — prevents silicone tube from tangling.

**Time per emitter:** 10–16 seconds depending on seed count.
**Speed for 100-emitter row:** ~25–30 minutes.

---

### Depth Control

Depth is set by the arm mounting height above soil. The solenoid stroke is fixed at 10mm — the cone housing punches 7mm into soil (3mm clearance gap + 10mm stroke).

```
Tip assembly cross-section:

  [solenoid body] ← fixed to arm end
        │ plunger (6mm shaft)
  [hollow nose housing] ← attached to plunger, moves with it
        │
  ──────┼────── soil surface
        │ 7mm into soil
        ▼
```

**Depth adjustment:** set by how high the arm is mounted relative to soil — lower mount = deeper penetration of cone. For prototype, 7mm is sufficient for sesame and gram in loose soil.

**Soil resistance:** the nose is a blunt ~17mm frustum (a 15mm seed bore leaves no room for a sharp point). In loose soil this punches a ~17mm hole needing roughly 2–4N — so the JF-0530B's 5N gives ~1.5× margin (not the 7× of a thin tip). If soil is firmer, reduce penetration to 3–5mm via arm height, or the nose may stall. This margin is the main cost of upsizing the path for large seeds.

---

### Seed Metering — Revolver-Style Rotating Pocket Gate

Seeds are metered by **geometry, not timing**. A solid cylindrical drum with one small pocket (recess) rotates inside a snug bore — exactly like a revolver cylinder or gumball wheel. The pocket captures a fixed volume of seed, carries it around in a closed chamber, and drops it out the bottom. The seed is never squeezed between closing surfaces, so it cannot be crushed.

```
Hopper cross-section:

  ┌─────────────────────┐
  │   seed bulk storage  │
  │      (gravity fed)   │
  │  [vibration motor]   │ ← pulses ~1s per fill to prevent bridging
  ├─────────────────────┤
  │  [feed throat]       │ ← funnels seeds toward pocket (fixed)
  ├─────────────────────┤
  │  ╔═══════════════╗   │
  │  ║  pocket drum  ║   │ ← rotates on SG90 D-shaft, 180° each way
  │  ║   ┌─────┐     ║   │   pocket sized per seed (swappable drum)
  │  ║   │  ●  │     ║   │   solid rim seals bore everywhere else
  │  ║   └─────┘     ║   │
  │  ╚═══════════════╝   │
  └─────────────────────┘
         │
     [silicone tube 14mm ID] ← runs externally alongside arm (not through it)
         │             flexes freely as arm rotates
         │
     [tube socket on tip housing] ← tube pushes IN here (full lumen)
```

**Cycle (one seed per cycle):**
1. **Default/idle** — pocket at bottom (empty), drum's solid rim blocks the column. Nothing falls (safe state).
2. **Fill** — SG90 rotates pocket up under the column → gravity drops seed(s) into the pocket. Solid rim now seals the exit below.
3. **Release** — SG90 rotates pocket back down to the outlet → seed drops into the silicone tube. Solid rim swings up and re-blocks the column.
4. **Return** — pocket sits empty at bottom = back to default. Ready for next cycle.

During rotation the pocket faces sideways and the bore wall acts as a lid — the seed rides in a closed chamber until it reaches the outlet. That is what makes it damage-free.

**Count is set by pocket geometry, sized per crop:**
- **Groundnut** — pocket = 1 kernel → drops 1 (occasionally 2, agronomically fine)
- **Gram** — pocket sized for 1–2 → drops 1–2
- **Sesame** — tiny pocket → drops a small cluster (~3–5), intentional for germination odds

Because the count depends on pocket size, the pocket lives in a **swappable drum** — one drum per seed size (S/M/L). The SG90, housing, feed throat and bore are fixed; only the drum slides off the D-shaft and is swapped per crop. This replaces the old screw-in metering insert — still a single part to swap, just a slide-in drum instead of a threaded bore.

**Tube routing:** the 14mm-ID silicone tube connects from the drum outlet socket to the tip-housing socket — the tube **pushes into** both ports (not over a barb), keeping the full lumen open so large seeds pass. Runs outside the arm (solid PETG). Slack loop near the outlet absorbs 300° arm rotation without pulling taut.

**Why the seed path is large (14mm tube / 15mm bore):** sized for the biggest target seed (groundnut/maize ~13mm) so every crop passes the same hardware. Trade-offs accepted: a heavier, wider tip (more cantilever load on the stepper shaft), a blunt soil nose needing more punch force (~1.5× margin, see above), and 14mm-ID silicone tube which is industrial-grade (not aquarium air-line). For a small-seed-only build, these could all shrink — revisit if groundnut/maize are dropped.

**Bridging prevention:** the drum self-agitates the bottom of the column each rotation, but seeds can still bridge higher up in the hopper. A coin vibration motor on the hopper wall pulses ~1s during each fill step to break bridges and keep seeds flowing into the feed throat. Runs for **all crops** — the pulse is gentle and cannot harm small seeds (sesame/gram), and it cannot cause over-dispensing because the pocket meters by fixed volume regardless of flow rate. Wide hopper throat + feed funnel further reduce bridging.

---

## Problem 3: Crop-Specific Pattern Configuration

All patterns configurable in Farm OS mobile app. No hardware changes between crops.

### Pattern Definitions

```json
{
  "sesame": {
    "center_seed": true,
    "radius_cm": 10,
    "positions": 6,
    "angle_offset_deg": 0,
    "depth_cm": 1.5,
    "arm_height_mm": 15,
    "seeds_per_position": 1,
    "arm_speed_rpm": 6
  },
  "groundnut": {
    "center_seed": false,
    "radius_cm": 10,
    "positions": 4,
    "angle_offset_deg": 45,
    "depth_cm": 6,
    "arm_height_mm": 10,
    "seeds_per_position": 1,
    "arm_speed_rpm": 4
  },
  "tomato": {
    "center_seed": true,
    "radius_cm": 8,
    "positions": 3,
    "angle_offset_deg": 0,
    "depth_cm": 1.0,
    "arm_height_mm": 15,
    "seeds_per_position": 1,
    "arm_speed_rpm": 6
  },
  "onion": {
    "center_seed": false,
    "radius_cm": 6,
    "positions": 8,
    "angle_offset_deg": 0,
    "depth_cm": 1.0,
    "arm_height_mm": 15,
    "seeds_per_position": 1,
    "arm_speed_rpm": 8
  },
  "chili": {
    "center_seed": true,
    "radius_cm": 8,
    "positions": 4,
    "angle_offset_deg": 45,
    "depth_cm": 1.5,
    "arm_height_mm": 15,
    "seeds_per_position": 2,
    "arm_speed_rpm": 6
  }
}
```

**Custom pattern:** Farmer can define own geometry. App shows visual preview of pattern before confirming.

### Visual Pattern Examples

```
SESAME (1 center + 6 at 10cm radius):     GROUNDNUT (4 at 10cm radius):

           ●                                        ●
     ●     ●     ●                            
       ↘   ●   ↙                              ●   ●
         [drip]                                 [drip]
       ↗       ↖                              ●   ●
     ●           ●
                                                  ●

ONION (8 at 6cm radius):               TOMATO (1 center + 3 at 8cm):

        ●   ●                                   ●
      ●       ●                           ●   [drip]   ●
      ●  [drip]  ●                              ●
      ●       ●
        ●   ●
```

---

## Soil Quality Adaptation (AI Layer)

On top of the fixed pattern, real-time soil readings from the probe influence seeding decisions:

| Condition | Normal Pattern | Adapted Behavior |
|---|---|---|
| Low soil EC (<0.2 mS/cm) | Plant at configured spacing | Reduce to 50% of positions — poor soil supports fewer plants |
| High soil EC (>1.5 mS/cm) | Plant at configured spacing | Add 1–2 extra positions — rich soil supports denser planting |
| Very hard soil (high resistance) | Attempt seeding | Skip emitter — seed won't germinate in compacted soil |
| Very dry soil (moisture <15%) | Attempt seeding | Flag emitter as possible clog, skip |
| Optimal conditions | Configured pattern | Plant full configured pattern |

Adaptation applied per-emitter, not per-row. Row 4 emitter 6 may be treated differently from Row 4 emitter 7 based on localized readings.

---

## GPS Seeding Log

Every seeding event logged to Farm OS database:

```json
{
  "timestamp": "2026-04-12T06:47:23",
  "emitter_id": "R4-E6",
  "gps": { "lat": 10.234567, "lng": 77.891234 },
  "crop": "sesame",
  "pattern_executed": "center+6",
  "seeds_dropped": 7,
  "soil": {
    "moisture_pct": 68,
    "ec_mS": 0.8,
    "ph": 6.4,
    "temp_c": 28
  },
  "depth_cm": 1.5,
  "confidence": "high",
  "emitter_status": "active"
}
```

**Later correlation:** When germination monitoring (UC-05) runs, germination rate at each emitter GPS point is matched against seeding log. AI learns which soil conditions at seeding time predict germination success. Feeds back into next season's adaptive thresholds.

---

## Multi-Hopper Variant (Future)

For farms growing 2 varieties in adjacent rows or alternating emitters, a dual-hopper configuration routes different seeds to the same arm:

```
[Hopper A — Variety 1]    [Hopper B — Variety 2]
         │                          │
    [Gate servo A]             [Gate servo B]
         │                          │
         └──────────┬───────────────┘
                    │
              [common tube]
                    │
              [seeder arm]
```

UNO Q selects which gate drum to cycle per emitter based on: GPS position + pre-loaded variety map. Farmer maps which rows get which variety once in the app.

---

## Bill of Materials

| Component | Spec | Source | Est. Cost |
|---|---|---|---|
| Seeder arm body | Solid PETG, 10cm — no hollow channel needed | 3D print | Rs 25 filament |
| Stepper hub + mount | PETG, Double-D bore for 28BYJ-48 shaft | 3D print | Rs 20 filament |
| Stepper motor + driver | 28BYJ-48 5V + ULN2003 board, 4096 steps/rev | Robocraze | Rs 254 |
| Punch solenoid | JF-0530B 12V DC, 5N, 10mm stroke, spring return | Amazon.in | Rs 433 |
| Tip housing (hollow nose) | PETG, hollow bore 15mm, side tube socket, attaches to plunger | 3D print | Rs 25 filament |
| Gate servo (seed release) | SG90 micro — rotates pocket drum 180° each way | Robu.in | Rs 80–100 |
| Pocket drums (S/M/L) | PETG print, one per seed size, slide on SG90 D-shaft | 3D print | Rs 15 filament |
| Vibration motor | Coin type 1034, 3–4V | Robocraze | Rs 25 |
| IRLZ44N MOSFET | Logic-level N-ch, controls solenoid via D11 | Robu.in | Rs 39 |
| 1N4007 diode | Flyback protection — 1× solenoid, 1× vibration motor | Amazon.in | Rs 20 |
| Hopper body | PETG print, ~200ml capacity, feed throat + fixed drum housing at outlet | 3D print | Rs 45 filament |
| Silicone seed tube | 14mm ID — gate outlet socket to tip socket, ~130mm | Industrial supply | Rs 40 |
| Mounting hardware | M3 bolts, nuts, brass inserts | Local hardware | Rs 80 |
| **Total** | | | **~Rs 1,050** |

---

## Software Architecture

### UNO Q — STM32 MCU Side (Real-Time)
- Tube following PID loop (camera input → steering correction)
- Moisture sensor reading at 10Hz
- Peak detection state machine
- Arm stepper control (28BYJ-48 via ULN2003, JMISC GPIO 1–4)
- Step counting for arm angle (683 steps = 60°, max 300° forward then reverse)
- Solenoid MOSFET control (D11 — extend 300ms, retract on release)
- Gate drum control (A3 — SG90 rotates pocket 180°: fill→release→return per seed)
- Vibration motor pulse (D13 via MOSFET — ~1s during each fill step, all crops)
- Serial bridge to Linux side (event reporting)

### UNO Q — Linux/Qualcomm Side (AI)
- OpenCV camera processing (tube detection, bump detection)
- Soil reading fusion (moisture + bump → confidence scoring)
- Crop pattern lookup + adaptation logic
- GPS logging via UART
- WiFi sync to RPi5 server
- Calibration routines

### RPi5 Server
- Seeding log storage (SQLite)
- Germination correlation analysis (post-season)
- Pattern config management and sync to UNO Q
- Clogged emitter map generation
- Mobile app API

### Mobile App / WhatsApp
- Crop + pattern selection before run
- Tip spring confirmation
- Live progress (emitters completed / remaining)
- Clogged emitter alert
- Post-run summary report

---

## Pre-Run Checklist (Farmer Flow)

```
1. Attach seeder arm to rear mount          (30 seconds)
2. Fit pocket drum for the crop (S/M/L):    (~2 minutes, bench)
   undo 2 SG90 screws → lift SG90+drum out → swap drum on horn → refit
3. Fill hopper with seeds                   (2 minutes)
4. Connect silicone tube: push into drum outlet socket + tip socket (10 seconds)
5. Connect solenoid wires to arm cable      (10 seconds)
6. Open Farm OS app
7. Select: Crop type → Sesame
8. App confirms: Pattern = center+6, Radius = 10cm, Drum = S, Depth = arm height
9. Run 10-seed calibration (robot stationary, drop 10 seeds, count) → Confirm
10. Navigate robot to start of first drip row
11. Press START
```

Robot runs fully autonomously from that point. Farmer can monitor progress in app or leave.

---

## Arm Support — Backup (build ONLY if the arm sags)

The seeder arm (100mm) cantilevers off the 28BYJ-48 stepper shaft, carrying the solenoid + tip (~120g) at the far end. The stepper's output shaft runs in a small plastic gearbox bushing not designed for sideways/overhung load. Possible effects:
- **Static sag** at the tip ≈ 1–2mm (borderline vs the ±2mm seeding tolerance).
- **Cyclic wear** — the punch reaction pushes the tip up/down each cycle, slowly wearing the bushing so sag grows over time.

**Verdict:** moderate, not urgent. For a prototype/demo it may be fine as-is. **Do not build the support pre-emptively** — print and assemble the arm, run it, and only add support if you actually see sag or wobble.

### If support is needed — lazy-susan turntable bearing

A turntable (lazy-susan) bearing concentric with the stepper takes the arm's weight, the punch uplift, and the tilting moment off the fragile stepper shaft — with low rolling friction (important because the 28BYJ-48 is weak). It's the same idea as a **restaurant table spinner / microwave turntable**.

How it handles both directions: the bearing's two plates are **captured** (crimped around the ball ring) so they spin but can't separate — resisting **down** (gravity) and **up** (punch reaction). The offset arm load becomes a down/up couple across the ball ring, which the captured raceway resists → no tilt.

```
   ARM CARRIER DISC (print) ── bolts to bearing TOP plate corners
   ═══════════════════════
   TURNTABLE BEARING (buy)  ◯ ← open center
   ═══════════════════════
   BASE PLATE (print) ──────── bolts to bearing BOTTOM plate corners
        │
   stepper shaft passes UP through the open center → drives the carrier
   (28BYJ-48 supplies torque only; bearing carries all the load)
```

**Buy:**
| Item | Spec | Cost | Where |
|---|---|---|---|
| Lazy-susan turntable bearing | **3 inch (75mm), square steel, OPEN center, captured raceway** | Rs 100–250 | [Amazon.in B0BY9FYDBD](https://www.amazon.in/dp/B0BY9FYDBD) (HEAVY DRIVER 3" turntable) |
| M4 bolts + nuts | ~8 pcs — match the bearing's corner holes (verify M4 vs M5) | Rs 30 | Local / Amazon |

> Confirmed suitable from product photo: open center (shaft passes through) + captured plates (spin but don't pull apart → resists punch uplift). No screws included — buy bolts separately.

**Print (2 flat discs, ~40g total):**
- **Arm carrier disc** (top) — bolts to bearing top plate; arm + stepper hub attach here; rotates.
- **Base plate** (bottom) — bolts to bearing bottom plate; holds stepper centered below; fixes to seeder frame; static.

**Sequence to avoid rework:** buy the bearing first → measure its actual hole pattern + center → *then* design/print the two discs to fit it. Do not print the discs from guessed dimensions. SCAD for these is not yet created (deferred until the support is actually needed).

---

## Known Limitations & Mitigations

| Limitation | Mitigation |
|---|---|
| Moisture sensor unreliable in very wet soil (post-rain) | Fall back to visual bump detection only. Delay seeding 2hrs after heavy rain. |
| Tube buried more than 2cm | Robot uses emitter spacing estimation (measured during calibration) to predict emitter positions |
| Seed bridging in hopper (clumped seeds) | Vibration motor pulses ~1s during each fill step (all crops). Drum rotation also self-agitates the column base. Wide hopper throat minimizes bridging. |
| Arm positioning error (±2mm) | Acceptable for 10cm radius pattern. Seeds germinate and spread beyond exact placement. |
| Seed lodged in tip housing bore | Bore 15mm + tube 14mm pass every target seed (groundnut ≤13mm). Full lumen kept by socket joints (no barb choke). Vibration motor helps clear any stuck seed. |
| Very hard soil (rocky patch) | Blunt ~17mm nose needs ~2–4N; JF-0530B 5N gives ~1.5× margin in loose soil. If nose can't penetrate, emitter flagged and farmer notified. Lower arm mounting / reduce depth for firmer soil. |
| Silicone tube tangle | Arm sweeps max 300° forward then always reverses to 0°. 300° twist is within silicone's comfortable flex range. Slack loop at gate end prevents pull. |
| Battery drain from vibration motor | Motor pulses ~1s per fill step only — not continuous. Negligible power increase (<2%). |
| Solenoid heat on long runs | JF-0530B duty cycle: 300ms on per planting event, ~10s off between positions. Average duty <5% — no heat buildup. |

---

## Future Enhancements

| Enhancement | Complexity | Impact |
|---|---|---|
| Dual hopper (2 crop varieties) | Medium | Mixed-variety rows per farmer preference |
| Adjustable arm length | Low — servo-controlled telescoping | Variable radius per emitter without hardware swap |
| Fertilizer micro-dosing at planting | Medium — second tube + pump | Place slow-release fertilizer at each seed point |
| Germination feedback loop | Low (software only) | Per-emitter seeding parameters auto-tune based on last season's germination data |
| Soil pH auto-adjust (lime/sulfur micro-dose) | High | Adjust soil pH per emitter at seeding time based on probe reading |
