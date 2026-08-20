# Bill of Materials — as built

Everything physically fitted to the robot that produced the demo, as of **2026-08-19**. The
circuit schematics are in [`schematic/`](schematic/); the seeder
mechanism is in [`seeder.md`](seeder.md).

Prices are indicative Indian retail (₹, 2026).

## Compute & sensing

| Item | Spec / pins | Qty | ₹ |
|---|---|---:|---:|
| **Arduino UNO Q** | 4 GB RAM / 32 GB eMMC (ABX00173) — **the only computer** | 1 | 6,000 |
| **QHM-999RL USB webcam** | SunplusIT `0806:0806`. **The robot's vision sensor.** Captured at 320×240 YUYV @ 30 fps; sees **0.43 m** of ground | 1 | — |
| **MPU-6050 IMU** | 6-axis; **only the gyro's Z axis is used**, for closed-loop heading. **A4 = SDA, A5 = SCL** via `Wire2` | 1 | ~150 |
| ESP32-CAM | OV2640 — **superseded by the USB webcam**; kept for the record | 1 | 589 |

**The camera and the gyro are the whole of the robot's sensing.** There are no soil probes and no
GPS fitted. The UNO Q is the whole compute stack — no companion PC, no cloud inference.

<details>
<summary><b>Why a USB webcam replaced the ESP32-CAM, and two things about it that will break a run</b></summary>

The ESP32-CAM delivered **0.7–6.7 fps, erratic**. A robot creeping at 0.17 m/s travels **24 cm**
between frames at 0.7 fps, which no control gain can rescue — the loop was starved, not badly
tuned. The QHM-999RL delivers a steady **29.8 fps** over USB. Full measurements and the reason a
passive OTG adapter cannot work: [`usb-camera.md`](usb-camera.md).

- It has **continuous autofocus and powers up with it ON**, which is pure liability on a camera at
  a fixed height. Autofocus is disabled and focus pinned by a **udev rule** so it survives a replug
  (`scripts/99-farmcam-focus.rules`).
- **`/dev/video` indices are not stable across boots.** The camera is found by matching its name in
  sysfs, never by a hardcoded index.
</details>

## Drive

| Item | Spec / pins | Qty | ₹ |
|---|---|---:|---:|
| 4WD chassis + gear motors | 100 mm wheels, 6 mm shaft | 1 | — |
| IBT-2 motor driver (BTS7960) | one per side, 2 motors each. **Left**: RPWM D3, LPWM D5, R_EN D7, L_EN D8. **Right**: RPWM D6, LPWM D9, R_EN D4, L_EN D12 | 2 | — |

<details>
<summary><b>Two PWM facts that each cost a session</b></summary>

PWM is available on **D3, D5, D6, D9** only, and calling `pinMode()` on a PWM pin *before*
`analogWrite()` kills PWM on that pin (outputs ~0 V).
</details>

## Seeder

| Item | Spec / pins | Qty | ₹ |
|---|---|---:|---:|
| **S3003 servo** | spool / arm rotation — **D10**. Calibrated: physical 90° ← servo command 64 | 1 | ~250 |
| **SG90 micro servo** | turns the metering drum — **D11** | 1 | 90 |
| **Hopper + pocket drums** | printed hopper feeding a pocket drum that meters by volume. **One drum per seed size** — slide it off the servo horn and fit another, no other change | 1 + drums | — |
| **JF-0530B solenoid** | push-pull, **12 V**, 5 N, 10 mm stroke, spring return — the punch. **One fitted, at one end of the arm** | 1 | 433 |
| IRLZ44N MOSFET | logic-level N-ch, solenoid low-side driver — gate on **A3** via 100 Ω | 1 | 39 |
| 1N4007 diode | flyback across the coil, **band (cathode) to +12 V** | 1 | ~3 |
| Gate resistors | **100 Ω** A3→gate series, **10 kΩ** gate→GND pulldown | 2 | ~2 |
| Silicone tubing | 14 mm ID — passes a groundnut | 15 cm | ~50 |
| **Lazy-susan bearing** | carries the rotating arm — takes the punch load off the servo | 1 | — |
| M3 hardware / brass inserts | arm + tip assembly | — | ~200 |

3D-printed parts (PETG/PLA): seeder arm body, spool hub, hopper, pocket drums (one per seed
size), hollow tip housing.

<details>
<summary><b>Why the servos are on digital pins, and one call that hangs the MCU</b></summary>

**Servos work only on DIGITAL pins on the UNO Q** — A3 gives no motion at all, which is why the
spool and drum sit on D10/D11 and A3 was left to the MOSFET gate. Also: `Servo.detach()` /
`attach()` at runtime **hangs the MCU**, taking the RouterBridge down with it.
</details>

## Power — and its protection

| Item | Spec | Qty | ₹ |
|---|---|---:|---:|
| 3S LiPo | ~12.6 V full | 1 | — |
| Buck converter | 12 V → 5 V, feeds the UNO Q 5 V rail + servos | 1 | — |
| **Main fuse** | **20 A** blade + inline holder, on LiPo+ before the block | 1 | ~80 |
| **Branch fuses** | **2 A** solenoid, **3 A** buck input, + holders | 2 | ~200 |
| Terminal / distribution block, XT60 | — | 1 | ~150 |
| Battery-sense divider | 10 k / 2 k + 100 nF at the node → A4 — **currently disconnected**, see below | 1 | ~10 |
| **1 mm² stranded wire** | ≈17 AWG, all power branches | 2 m | ~130 |

> ⚠️ **Every power branch needs its own fuse, sized to its own wire.** Fuse for the fault current,
> gauge for the running current.

<details>
<summary><b>The fire, and why the 20 A main fuse did not stop it</b></summary>

A fuse protects the thinnest wire downstream of it, not the load. A 20 A main fuse on a Dupont
jumper (~1.5 A) is not protection — the wire becomes the fuse. On **2026-08-15** the 12 V solenoid
feed shorted while the battery was being moved and **caught fire; the 20 A main fuse never blew.**
Nothing was electrically wrong: the solenoid had worked for days, the diode and MOSFET were both
fine. Full analysis in [`troubleshooting.md`](troubleshooting.md) → `[Power]`.

**Power the UNO Q from its 5 V header pin, not USB.** USB creates a ground loop that corrupts the
ADC, and VBUS keeps the board from staying powered off.
</details>

## Two pins the gyro cost us

The MPU-6050 had to take **A4/A5**, which were already spoken for. What that cost:

| Casualty | Was | Now |
|---|---|---|
| **Battery monitor** | 10 k/2 k divider → A4 | `BATT_PRESENT 0`. `getBattery` answers `present:false` instead of reporting a floating pin. **Runs must pass `nobatt`.** |
| **Gate sense** | `SENSE_PIN` = A5 | `GATE_SENSE 0` — `analogRead` on A5 would re-mux the pad and kill the I2C bus mid-run |
| **Motor current sense** | `IS_L_PIN` = A5 | `CURRENT_SENSE 0` — never wired |

The trade was worth it: heading went from ±5–10° of open-loop guesswork per turn to **0.3° worst
error across four consecutive 90° turns.**

<details>
<summary><b>Why the gyro had to take those two pins</b></summary>

The pins silk-screened **SDA/SCL** on the UNO Q header (D20/D21) have **no I2C peripheral behind
them** on this core — the board overlay hands those pads to USART3 and leaves `i2c2` disabled. So
`Wire.begin()` succeeds, every transfer fails, and a bus scan finds nothing no matter how correct
the wiring is. `Wire2` (i2c3) is on **A4/A5**, and that is the only solderable I2C on the board.
</details>

## Evaluated and deliberately NOT used

| Considered | Why rejected |
|---|---|
| **GPS + compass (BN-880)** | Consumer GPS is ±2–5 m; the demo plot is 3–6 m across, so it would be **less** accurate than dead reckoning, and it gives no heading at rest. The problem is plot-scale *relative* odometry, not absolute position. |
| **Stepper (28BYJ-48) for the arm** | Superseded by the S3003 servo — absolute positioning, fewer pins, no driver board. |
| **Header SDA/SCL pins for I2C** | Not a choice — they are electrically dead for I2C on this core. See above. |
