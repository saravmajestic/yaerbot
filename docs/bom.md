# Bill of Materials — as built

Everything physically fitted to the robot that produced the demo. Prices are indicative
Indian retail (₹, 2026). Wiring detail and pin assignments are in
[`uno-q-wiring.md`](uno-q-wiring.md); the seeder mechanism is in [`seeder.md`](seeder.md).

## Compute & sensing

| Item | Spec | Qty | ₹ |
|---|---|---:|---:|
| **Arduino UNO Q** | 4 GB RAM / 32 GB eMMC (ABX00173) — **primary board** | 1 | 6,000 |
| ESP32-CAM | OV2640, WiFi, 5 V — drip-line vision | 1 | 589 |
| Capacitive soil moisture sensor | Analog v1.2, corrosion resistant | 2 | 110 |
| DS18B20 temperature probe | 1-Wire, waterproof, stainless | 1 | 48 |
| Soil EC probe | DIY — 2× M3 stainless screws + divider, AC-driven from D13 | 1 | ~75 |

The UNO Q is the whole compute stack: the Qualcomm Linux side runs the web console, the
on-device LLM planner and the Edge Impulse vision model; the STM32U585 MCU does
real-time GPIO. No companion PC, no cloud inference.

## Drive

| Item | Spec | Qty | ₹ |
|---|---|---:|---:|
| 4WD chassis + gear motors | 100 mm wheels, 6 mm shaft | 1 | — |
| IBT-2 motor driver (BTS7960) | one per side, 2 motors each | 2 | — |

## Seeder

| Item | Spec | Qty | ₹ |
|---|---|---:|---:|
| **S3003 servo** | spool / arm rotation — **D10** | 1 | ~250 |
| **SG90 micro servo** | 2-pocket metering drum — **D11** | 1 | 90 |
| **JF-0530B solenoid** | push-pull, **12 V**, 5 N, 10 mm stroke, spring return — the punch | 1 | 433 |
| IRLZ44N MOSFET | logic-level N-ch, solenoid low-side driver — gate on **A3** via 100 Ω | 1 | 39 |
| 1N4007 diode | flyback across the coil, **band to +12 V** | 1 | ~3 |
| Silicone tubing | 14 mm ID — passes a groundnut | 15 cm | ~50 |
| Compression springs / M3 hardware / brass inserts | arm + tip assembly | — | ~200 |

3D-printed parts (PETG/PLA, ~110 g total): seeder arm body, spool hub, drum, hollow tip
housing, motor mounts, ESP32-CAM housing.

## Power — and its protection

| Item | Spec | Qty | ₹ |
|---|---|---:|---:|
| 3S LiPo | ~12.6 V full | 1 | — |
| Buck converter | 12 V → 5 V, feeds the UNO Q 5 V rail + servos | 1 | — |
| **Main fuse** | **20 A** blade + inline holder, on LiPo+ before the block | 1 | ~80 |
| **Branch fuses** | **2 A** solenoid, **3 A** buck input, + holders | 2 | ~200 |
| Terminal / distribution block, XT60 | — | 1 | ~150 |
| Battery-sense divider | 10 k / 2 k + 100 nF at the node → **A4** | 1 | ~10 |
| **1 mm² stranded wire** | ≈17 AWG, all power branches | 2 m | ~130 |

> ⚠️ **Every power branch needs its own fuse, sized to its own wire.** A fuse protects
> the thinnest wire downstream of it, not the load. A 20 A main fuse on a Dupont jumper
> (~1.5 A) is not protection — the wire becomes the fuse, and on 2026-08-15 one caught
> fire. **Fuse for the fault current, gauge for the running current.** Full analysis in
> [`troubleshooting.md`](troubleshooting.md) → `[Power]`.

## Evaluated and deliberately NOT used

Recording these because the reasoning is part of the design.

| Considered | Why rejected |
|---|---|
| **GPS + compass (BN-880)** | Consumer GPS is ±2–5 m; the demo plot is 3–6 m across, so it would be **less** accurate than dead reckoning, and gives no heading at rest. The problem is plot-scale *relative* odometry, not absolute position. |
| **Stepper (28BYJ-48) for the arm** | Superseded by the S3003 servo — absolute positioning, fewer pins, no driver board, and the UNO Q has exactly one free GPIO to spare. |
| Companion Raspberry Pi | Unnecessary: the UNO Q runs the LLM, the CV and the web console itself. |

## Identified for the next revision

Not fitted for this demo, but they are the fix for the known open-loop limits:

| Item | Fixes |
|---|---|
| MPU-6050 / IMU | closed-loop heading — a skid-steer pivot turn rotates by scrubbing the wheels sideways, so its calibration changes with the ground. A gyro removes the whole class of problem. |
| Wheel encoders (HC-89 slot + printed disc) | measured distance instead of timed dead reckoning |
| ADS1115 | 16-bit ADC — the UNO Q's analog inputs are 0–3.3 V and easily disturbed |
