# Farm OS — Drive Precision: straight lines and accurate turns

Why the robot doesn't drive straight or turn to an exact angle, what we measured, and
what actually fixes it. Written after a long debugging session on 2026-08-09..11 that
found **three independent faults** stacked on top of each other, plus one property of
skid-steer robots that no amount of tuning removes.

Read this before "fixing the trim" — trim was the wrong answer three times out of four.

---

## TL;DR

| Want | Answer |
|---|---|
| Straight hops | trim + all four wheels gripping. A gyro removes the need for trim entirely. |
| Accurate turns | **not achievable open-loop** beyond ~±5°. Lower turn duty + a deceleration ramp help; an IMU is the real fix. |
| Accurate hop distance | timing works if dead time is calibrated; wheel encoders make it exact. |

**One rule:** before adjusting any control parameter, prove the command reached the motors.
`getDiag` (see below) does that in one line.

---

## The faults we found, in the order they mattered

### 1. Loose IBT-2 signal wiring (worst, and invisible)

Dupont jumpers on the IBT-2 signal header **degrade progressively under vibration**. The
signature is nasty because it isn't a clean failure:

- Early in a session: `fwd 180 4s` → 2.60 m, dead straight
- An hour and a dozen runs later: **the same command drove a half circle**

Because it drifts, every calibration taken during that window is worthless, and any
parameter fitted to it will not converge — we watched a "required trim" climb 1.24 → 1.33
while chasing it. **Solder the signal lines and strain-relieve them to the chassis.**

### 2. One wheel not touching the ground

A **rigid 4-wheel chassis is over-constrained** — like a wobbly table it rests on 3 of its
4 contact points unless the axles are coplanar. On this robot two motor shafts sat ~1 mm
proud of the others; one gripped fine and the other floated, because *which* corner floats
depends on how the frame is twisted, not on height alone. 1 mm matters because a rubber
tire only deflects about that much under a light robot.

Effects, all of which look like other problems:
- the side with the floating wheel loses ~half its traction → large veer
- that side cannot counter-rotate → **turns become forward arcs instead of pivots**
- forward dead time measured **0.524 s** (it was really 0.104 s — the drag inflated it)

Fix: loosen that motor bracket, let the wheel settle on a flat surface, retighten; shim
1 mm if the frame is twisted. Verify all four wheels take load (rock the robot diagonally;
try sliding paper under each wheel).

> **On soil this matters much less** — soft ground conforms to a millimetre and all four
> wheels find grip. Some of what we chased is a hard-floor artifact. Re-test on real
> ground before doing mechanical surgery.

### 3. Normal motor variance — not a fault

Two nominally identical DC gearmotors run **5–15% apart at the same PWM**. It is
manufacturing tolerance (internal resistance, brush friction, gearbox drag) and every
hobby 4WD robot has it. Measured here by counting free-spinning wheel revolutions:
**5 left vs 4.75 right → 5%.** That is normal, and it is what trim is for.

### 4. Rotational coast — a property, not a fault

A skid-steer turn is low-friction, so the chassis keeps spinning after the brake:

| Turn duty | Coast |
|---|---|
| PWM 180 | **~75°** |
| PWM 120 | **~38°** |

At PWM 180 a 90° turn is *mostly* coast, so timing it is hopeless. **Turn at a lower duty.**

---

## The measurement that unlocked it

Two numbers that should have agreed, didn't:

| Condition | Right-side deficit |
|---|---|
| Free-running (revolution count; motor terminals 6.72 V vs 6.64 V = 1.2%) | **5%** |
| Driving under load (inferred from veer geometry) | **25%** |

**Free-running measurements hide load-dependent faults** — no current flows, so nothing
sags and nothing slips. Always take the loaded measurement too (push the robot against a
wall and probe the motor terminals). The 20-point gap was traction, not electrics.

---

## Calibration procedure (works, ~10 minutes)

Use `scripts/field_test.py` on the board. Recalibrate after **any** mechanical or wiring
change — every value below is surface-, battery- and hardware-specific.

```bash
FT="ssh unoq docker exec motor-control-main-1 python3 /app/python/field_test.py"

# 0. sanity: does the MCU receive what we send?
$FT diag

# 1. forward: two durations -> cruise speed AND dead time
$FT fwd 180 4 0.75      # measure distance
$FT fwd 180 1.5 0.75    # measure distance
$FT solve 4 <d_long> 1.5 <d_short>

# 2. turn: two durations at the LOW turn duty -> rate and coast
$FT turn 120 1.5 right  # measure angle
$FT turn 120 0.8 right  # measure angle
$FT tsolve 1.5 <a_long> 0.8 <a_short>

# 3. validate one seeding row
$FT row total=1.2 hop=0.4 turn=90 dir=right \
      speed=<s> startup=<st> ltrim=<lt> tpwm=120 tdps=<r> tstartup=<c> nobatt

# 4. validate the ROW CHANGE (two same-direction 90s + the gap)
$FT uturn leg=1.0 gap=0.4 turn=90 dir=right <same params>
```

**Two models, opposite signs — don't mix them up:**

```
forward:  time = startup_s      + distance/speed      startup_s  > 0   (dead time)
turn:     time = turn_startup_s + angle/tdps          turn_startup_s < 0   (coast)
```

A dead time means *nothing happens at first*; a coast means *it keeps going after you
stop*. If `tsolve` hands you a negative dead time, that's a coast — expected for turns.

### Reference numbers (2026-08-11, hard floor, 3S ~12 V, after fixes 1 and 2)

| Parameter | Value |
|---|---|
| `ltrim` | 0.75 (left 135 / right 180) |
| `speed` | 0.616 m/s @ PWM 180 |
| `startup` | 0.104 s |
| `tpwm` | 120 |
| `tdps` | 51 °/s |
| `tstartup` | −0.75 s (≈38° coast) |

→ a 40 cm hop is **0.75 s**; a 90° turn is **1.01 s**.

---

## Measure the thing you can actually measure

Most of our bad data came from eyeballing angles. Reframe the measurement:

| Don't measure | Measure instead | Why |
|---|---|---|
| "is that 90°?" | **a full/three-quarter circle** | back-to-parallel is easy to judge; 90° is not |
| turn angle in degrees | **lateral offset in cm** over a known distance | tape measure, not judgement — then compute the angle |
| "did it veer?" | offset **per metre** | comparable across runs of different length |
| each 90° of a row change | **are the two legs parallel** | that's the output that matters operationally |

Angle from offset, for a constant-radius arc over path length `s`:

```
(s/θ)(1 − cos θ) = offset          → solve θ
R = s/θ ;  speed difference = track_width / R   (as a fraction of average speed)
```

Worth building: tape a laser pointer to the chassis and mark a wall, or lay a printed
protractor under the pivot point.

---

## Software solutions

### Implemented

- **Dead-time model on forward** (`startup_s + d/speed`). Without it a 40 cm hop is issued
  as a sub-dead-time burst that barely translates — this is why an early boustrophedon
  attempt "rotated randomly within a 30 cm square" instead of driving rows.
- **Coast model on turns** (negative `turn_startup_s`), with the sleep clamped at zero:
  a turn smaller than the coast angle is simply not achievable at that duty.
- **Separate turn duty** (`turn_pwm`) — forward wants 180, turns want 120. One shared duty
  cannot express both.
- **Deceleration ramp** (`turn_ramp_s`, default off) — steps duty down to a floor over the
  last fraction of a turn, then brakes. Coast scales with spin speed at brake time, so
  easing off first shrinks the coast *and* its run-to-run scatter. There is still only one
  stop, and it happens slowly. Calibration is unaffected as long as the ramp is fixed.
- **Battery compensation** — scales duty by `V_cal/V_now` to hold the calibrated speed
  (symmetric: boosts a sagged pack, backs off a fresh one; clamped 0.75–1.5×).
- **`getDiag` self-checking** — after every move, compares what we sent against what the
  MCU latched and drove, and prints a `!!` banner on mismatch. Catches a broken command
  path *before* you start blaming control parameters.

### Available, not yet done

- **Fewer turns.** Every turn is an error source. Run rows along the **long** axis of the
  plot. A serpentine alternates turn direction between ends, so systematic overshoot
  partly cancels across a full cycle — but *not* within one row change (see below).
- **Auto-calibrate per surface.** Once an IMU is fitted, command a known 360° at the start
  of a run, measure it, and derive that surface's rate and coast. Removes
  "calibrated indoors, ran outdoors" completely.
- **Closed-loop heading** — the real prize; see hardware.

### The row change is the precision bottleneck

`A → B` (row), `B → B1` (**90°, row gap, 90°**), `B1 → C` (next row). **Both turns go the
same way**, so a *systematic* per-turn overshoot **doubles**: 10° per turn leaves the next
row 20° skew. Consequences:

- Tune the row change **as one primitive** (`field_test.py uturn`), not as two 90° turns.
  Its two outputs are separately measurable — heading (are the legs parallel?) and lateral
  (is the spacing the row gap?) — so two measurements pin two parameters.
- The row gap here equals the **seeder arm's outlet spacing**, so lateral error shows up
  directly as wrong seed spacing between rows.

---

## Hardware solutions, ranked by value per rupee

> **Parts identified to close these gaps are listed in [`bom.md`](bom.md)** → "Identified for the next revision".
> Short version of what changed after checking the UNO Q's own capabilities: the board
> has **no onboard IMU**, but it has a **Qwiic connector**, and Arduino's
> **Modulino Movement** (ABX00101, LSM6DSOXTR 6-axis) is a plug-in IMU with **no
> soldering and zero GPIO cost** — preferred over a hand-wired MPU-6050 because
> hand-made joints are what have cost this project the most. For distance, **AS5600
> ×2 behind a Modulino Hub** (TCA9548A mux, needed because AS5600's address is fixed)
> keeps encoders off the GPIO budget, of which **exactly one pin is free**.

### 1. IMU — highest value

**MPU6050** on the free `SDA`/`SCL` pins (both untouched; `A5` is the only free analog pin).
Use the **gyro only**: integrate yaw rate and turn until the measured angle reaches target.
Gyro drift is irrelevant over a 1–2 s turn. Coast stops mattering because you measure the
result instead of predicting it.

```
target = 90°
while |target − measured| > 2°:
    duty = clamp(Kp × error, 80, 140)     # automatically slows near target
stop, settle, re-read; one micro-correction if still out
```

Surface-independent, battery-independent, self-correcting. **It also removes the need for
trim** — hold heading during a straight hop instead of guessing `ltrim`. For production use
a **BNO085/BNO055** (~₹1500–2500): on-chip fusion, absolute heading, no hand-rolled drift
maths. Mount on foam, away from the motors.

### 2. Wheel encoders — for distance, not for heading

Be clear what they buy: in a skid-steer the wheels **slip by design during a turn**, so
heading derived from wheel difference is unreliable exactly when you need it. Encoders fix
**hop distance** (replacing the timing model); the IMU fixes **heading**. Different jobs.

Cheap option: **LM393 slot sensor + 20-slot disc** (~₹80–150). Tie both channels per side,
one digital pin, poll it (~30–50 pulses/s at our speed — no interrupt needed). A ~65 mm
wheel with 20 slots gives ~1 cm per pulse. Best implemented as a `driveDistance(pulses)`
RPC so the MCU closes the loop itself.

### 3. Mechanical

- **Wheelbase vs track.** A skid-steer with a *long* wheelbase relative to its track (32 cm
  here) scrubs hard and resists turning. Aim for wheelbase ≤ track.
- **All four wheels loaded** — shim coplanar, or add a rocker/suspension for uneven ground.
- **Weight distribution.** 4-wheel skid-steers turn *worse* at 50/50 front/rear — they buck
  and scrub. Deliberately loading one axle improves turning.
- **Tires** matched, clean, not glazed; narrower scrubs less.
- **Solder power and signal**, strain-relieved. Dupont goes high-resistance under load —
  the same lesson as the battery divider and the motor drive.

### Production stack

```
distance  ← wheel encoders        (closed-loop hops)
heading   ← IMU gyro, fused       (closed-loop turns + straight-line hold)
position  ← RTK GPS               (only if the field needs absolute reference)
```

---

## Debugging lessons (the expensive ones)

1. **Verify the command path first.** `getDiag` proves what the MCU received and which pins
   it drove. Without it we spent a dozen runs tuning parameters against a loose wire.
2. **Free-running measurements hide load-dependent faults.** Take the loaded one too.
3. **Don't re-fit the model to each new reading.** Hold several hypotheses and pick the test
   that *discriminates* between them. A theory that changes with every data point isn't a
   theory.
4. **A parameter that won't converge is telling you something.** "Required trim" climbing
   1.24 → 1.33 → never straight meant the fault wasn't trim-shaped at all.
5. **Small voltage differences don't cause big speed differences.** 1.2% of volts buys ~1.2%
   of speed. To explain 20% you need ~1.3 V, not 0.08 V.
6. **Beware the compensating parameter.** `ltrim=0.75` masks an asymmetry rather than fixing
   it, so it drifts as the pack sags. Note which parameters are papering over hardware.
7. **Check the newest change first.** Newly-mounted wheels, freshly-soldered joints, a
   just-flashed sketch. And **back up firmware before overwriting it** — we lost the ability
   to A/B a flash because the only copy of the previous sketch was clobbered.

---

## See also

- [`bom.md`](bom.md) — parts identified to close these gaps
- `docs/farm-os/wheel-encoder-build.md` — build/mount/wire the distance encoder
- `docs/farm-os/troubleshooting.md` — connector, network and flashing failures
- `apps/farm-robot/docs/farm-os/uno-q-wiring.md` in the **ai-labs** repo — pinout as it
  stood then. Superseded: the IMU went on **A4/A5** (`Wire2`), not the header SDA/SCL pins,
  which have no I2C peripheral behind them. Current sheets: `docs/schematic/`
- `firmware/farm_os/farm_os.ino` — `getDiag`, optional IBT-2 current sense (`CURRENT_SENSE`)
- `scripts/field_test.py` (yaerbot) — `diag`/`fwd`/`turn`/`solve`/`tsolve`/`row`/`uturn`/`plan`
