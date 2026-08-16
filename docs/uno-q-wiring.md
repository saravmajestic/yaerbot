# Farm Robot — UNO Q Wiring Reference
### Two-Box Layout: Circuit Box + Seeder Box

> Replaces RPi5 + Arduino Nano setup — see `rpi5-wiring.md` for previous
> UNO Q has two processors: STM32 MCU (owns all GPIO, real-time) and Qualcomm Linux (AI + server)

---

## Physical Layout

The robot chassis splits into two zones (see `blueprint.md §1.1a`):

```
←──────────────── 400mm ────────────────→
┌─────────────────────────────────────────┐
│  ┌──────────────────┐ ┌───────────────┐ │
│  │                  │ │               │ │
│  │  CIRCUIT BOX     │ │  SEEDER BOX   │ │
│  │                  │ │               │ │
│  │  UNO Q           │ │  S3003 (spool)│ │
│  │  IBT-2 × 2       │ │  SG90 (drum)  │ │
│  │  XY-3606         │ │  Solenoid(s)  │ │
│  │  EC 555 circuit  │ │  + MOSFET(s)  │ │
│  │  MOSFET          │ │  Hopper       │ │
│  │  BN-880 GPS      │ │  Arm          │ │
│  │  Breadboard      │ │               │ │
│  └──────────────────┘ └───────────────┘ │
│       front →              ← rear       │
└─────────────────────────────────────────┘
        10-pin JST cable (seeder)
```

> **v2 seeder (Option A):** the rotating-spool design drops the **28BYJ-48
> stepper + ULN2003** entirely. The **S3003** drives the spool through a 1:1
> gearbox; the **SG90** meters the 2-pocket drum; one (later two) **JF-0530B
> solenoid** punches. Two servos, no stepper. See `3d-models/v2/PRINT_v2.md`.

**Why two boxes:**
- Seeder is a swappable attachment — disconnect cable, lift off seeder box, swap for soil probe holder
- Circuit box never changes between runs
- All electronics protected from soil and vibration in separate enclosures

---

## Inter-Box Cable — v2 (2-servo seeder)

The only connection between the two boxes. A single **10-pin JST-XH** still fits,
now carrying two servo signals (no stepper) + dual-solenoid + doubled power.

| Pin | Wire | From (circuit box) | To (seeder box) | Purpose |
|---|---|---|---|---|
| 1 | 5V power | UNO Q 5V rail | S3003 + SG90 VCC | Servo power |
| 2 | GND | UNO Q GND | Servo GND + solenoid MOSFET source | Common ground |
| 3 | 12V power | LiPo direct (fused) | Solenoid(s) (+) via 1N4007 cathode | Solenoid power — **12V, NOT the 5V rail** |
| 4 | Signal | **D10** (PWM) | S3003 signal | Spool drive (1:1 gearbox) |
| 5 | Signal | **D11** (PWM) | SG90 signal | Drum metering — servo needs a **digital** pin (see note) |
| 6 | Signal | **A3** | Solenoid #1 IRLZ44N gate (via 100Ω) | Punch #1 — on/off only, analog pin OK |
| 7 | Signal | **D13** | Solenoid #2 IRLZ44N gate (via 100Ω) | Punch #2 (dual-punch; spare until 2nd solenoid fitted) |

> ⚠️ **The solenoid must be fed from pin 3 (12V), never the 5V rail.** Found 2026-08-14
> after the punch silently stopped working: the `+` had been relanded on the buck's 5V
> output during a buck refit. Force goes as V², so 5.45V gave ~1.0N of the rated 5N —
> nothing moved, while the gate, the MOSFET and A3 all measured perfectly healthy. The
> `+12V` node is the solenoid `+` **and** the 1N4007 cathode together; move both, and
> check the diode band still faces +12V. See `troubleshooting.md` → `[Seeder]`.

> ⚠️ **Servos must be on DIGITAL pins on the UNO Q.** Bench-verified 2026-07-07: the
> Zephyr core drives `Servo` PWM only on timer-capable **digital** pins — **A3 gave no
> servo motion at all** (S3003 on D10 worked; SG90 was dead on A3, alive on D11). So SG90
> moved to **D11**, and solenoid #1 (a simple on/off MOSFET gate — no PWM needed) took the
> freed **A3**. Keep this rule: servos → digital pins; dumb on/off loads → analog pins.
| 8 | 5V power | UNO Q 5V rail | Servo VCC (shared) | Extra servo current |
| 9 | GND | UNO Q GND | Servo / solenoid return | Extra ground |
| 10 | spare | — | — | Future (2nd 12V / sensor) |

**Why doubled 5V/GND (pins 8/9):** v2 has *two* servos — S3003 under load can pull
~0.5–1A. One thin JST wire is marginal, so 5V and GND each get two pins.

**Wire gauge:** 22 AWG signal (4–7), 20 AWG power (1–3, 8–10).

**Connector choice:** JST-XH 10-pin is compact, locks, and is easy to source. A 10-pin
Dupont housing with a latching clip works for prototyping.

> **Vibration motor (dropped in v2):** v2's drum meters positively (pockets), so the
> de-jam vibration motor isn't needed. If the bottle/throat ever bridges, fit a small
> vibration motor on the **spare pin 10** + a 3rd MOSFET — but start without it.

### Soil-probe attachment — its OWN connector

The soil probe is the *other* swappable rear attachment (EC + moisture + temperature
at depth). It needs **sensor lines** — 3.3V, GND, analog ADC, 1-Wire — which are
**completely different pins** from the seeder's servo/solenoid PWM + 12V. So it does
**not** reuse the seeder cable; give it a **separate small connector**:

| Pin | Wire | UNO Q pin | Purpose |
|---|---|---|---|
| 1 | 3.3V | 3.3V | Sensor power (ADC accuracy) |
| 2 | GND | GND | Common ground |
| 3 | Analog | **A4** (spare) | Probe moisture / EC midpoint |
| 4 | Analog | **A5** (spare) | 2nd analog (EC / pH later) |
| 5 | 1-Wire | **D2** (shared bus) | Probe DS18B20 temp (addressable — shares the front bus) |

A **5-pin JST-XH** is plenty. Because the seeder and the probe are **mutually
exclusive** (only one mounted at a time), there's no conflict — you just plug whichever
attachment's connector. The seeder uses D10/A3/D11/D13; the probe uses A4/A5/D2 — no
overlap. (A single shared "superset" connector is possible but needs more wires for no
real benefit when the attachments never run together.)

---

## Circuit Box — Internal Wiring

Everything inside the front box.

### Power

```
[3S/4S LiPo XT60]
       │
       ├──── IBT-2 #1 B+/B−    (left motors, direct from LiPo)
       ├──── IBT-2 #2 B+/B−    (right motors, direct from LiPo)
       │
       └──── [XY-3606 buck converter]
                  Output: 5.0V  (verify with multimeter)
                  (+) → UNO Q 5V pin (Arduino header)   ← NOT USB-C (see warning)
                  (−) → UNO Q GND pin (Arduino header)
```

> ⚠️ **Power via the 5V HEADER PIN, never via USB-C.** Feeding the board through USB-C makes its VBUS a power-on signal — after a software `shutdown`/`poweroff` the PMIC sees VBUS and **immediately reboots** ("shutdown doesn't stay off"). Powering on the 5V pin avoids this AND frees the USB-C port for data/debug/mic. See `troubleshooting.md → [Power] Shutdown won't stay off`.

Common ground chain (all must share):
```
LiPo GND ──┬── IBT-2 #1 B−
            ├── IBT-2 #2 B−
            └── XY-3606 GND in → XY-3606 GND out → UNO Q GND
```

### Motor Control — IBT-2 × 2

**IBT-2 #1 — Left motors (FL + RL in parallel)**

| IBT-2 Pin | UNO Q Pin |
|---|---|
| RPWM | D3 |
| LPWM | D5 |
| R_EN | D7 |
| L_EN | D8 |
| VCC | 5V |
| GND | GND |
| B+ / B− | LiPo direct |
| M+ / M− | Left motors |

**IBT-2 #2 — Right motors (FR + RR in parallel)**

| IBT-2 Pin | UNO Q Pin |
|---|---|
| RPWM | D6 |
| LPWM | D9 |
| R_EN | D4 |
| L_EN | D12 |
| VCC | 5V |
| GND | GND |
| B+ / B− | LiPo direct |
| M+ / M− | Right motors |

> EN pins are software-driven by STM32 (HIGH on init, LOW on emergency stop) — not tied to 5V as in the old Nano setup.

### Soil Sensors

**Moisture sensors × 2**

| Sensor Pin | UNO Q Pin | Notes |
|---|---|---|
| VCC | 3.3V | Use 3.3V for ADC accuracy |
| GND | GND | |
| AOUT (sensor #1) | A0 | 14-bit, 0–16383. Dry≈12000, Wet≈4000 |
| AOUT (sensor #2) | A1 | |

Mount both sensors at front of chassis, probes facing down 1–2cm above soil.

**DS18B20 temperature (1-Wire)**

```
DS18B20 red   → 3.3V
DS18B20 black → GND
DS18B20 yellow → D2
               │
             4.7kΩ pull-up
               │
             3.3V
```

Solder the 4.7kΩ directly at the breadboard — mandatory, 1-Wire won't read without it.

**EC sensor — DIY, MCU-driven (no 555)**

The MCU makes the AC itself, so there's no 555 chip and no 5V→3.3V hazard (the drive
pin swings 0–3.3V → ADC-safe). One resistor, two electrodes.

```
DRIVE GPIO (free JMISC pin) ──► electrode A
                                   │
                                [ soil ]
                                   │
                    electrode B ──┬──► A2   (sense)
                                  │
                                 22kΩ
                                  │
                                 GND
```

- Firmware toggles DRIVE (HIGH→read A2, LOW→read A2; `EC_raw = v_high − v_low`) and
  **alternates polarity** each read so net DC ≈ 0 → no electrolysis (the job the 555
  used to do). Conductive soil → bigger `EC_raw`.
- Worst-case current (electrodes shorted) = 3.3V/22kΩ ≈ 0.15mA → safe for the pin, so
  no series resistor needed.

**Electrodes:** 2× M3 stainless screws bolted **through a plastic bar, 15mm apart** — a
nut on the back fixes the spacing (must stay constant or readings drift); clamp the wire
under a washer+nut (stainless doesn't solder). Tips ~20–30mm, inserted to a consistent
depth; insulate the top joints.

**GPS + Compass — BN-880**

```
BN-880 VCC → 5V
BN-880 GND → GND
BN-880 TX  → JDIGITAL Serial1 RX     ← NOT D0/D1
BN-880 RX  → JDIGITAL Serial1 TX
BN-880 SDA → I2C SDA (HMC5883L compass)
BN-880 SCL → I2C SCL
```

> D0/D1 are owned by RouterBridge (Linux↔MCU RPC). Nothing else connects there.

**Solenoid MOSFET driver (seeder box — sits next to the solenoid)**

> **DECISION (2026-07-25): ONE driver drives BOTH solenoids in parallel.** The two arm
> punches always fire **at the same instant** (both holes together), so they don't need
> independent switching — wire both solenoids in parallel off a **single** IRLZ44N driver on
> **A3**. Current doubles to ~1.5 A (2 × 0.75 A) — trivial for the IRLZ44N (tens of A rated);
> just size the +12 V and drain leads for ~1.5 A (1 mm² is plenty). This halves the parts and
> the soldering. **D13 stays reserved** — add a 2nd independent driver later only if you ever
> want to fire the two punches separately.
> Bench-verified 2026-07-25: single soldered driver fires the solenoid at full force on 12 V.

```
LiPo 12V (fused) ── 1N4007 cathode ── solenoid(s) (+)   ← both solenoids in parallel
                          │
                     1N4007 anode ── IRLZ44N Drain ── solenoid(s) (−)
                                     IRLZ44N Source ── GND (common)
                                     IRLZ44N Gate   ── 100Ω ── A3   ← punch (fires both)
                                     10kΩ gate→source pulldown
                                     (D13 reserved for a 2nd independent driver — not built)
```

Solenoid fires on its pin HIGH (12V through coil, rod extends). Pin LOW → spring
retracts rod. Both solenoids share the 12V (pin 3) and GND.

---

## Seeder Box — Internal Wiring

Everything inside the rear box. All power and signals arrive via the 10-pin cable.

```
[Pins 1 + 8 — 5V]  ──┬── S3003 VCC (red)
                     └── SG90  VCC (red)

[Pins 2 + 9 — GND] ──┬── S3003 GND (brown)
                     ├── SG90  GND (brown)
                     └── Solenoid MOSFET Source(s)

[Pin 4 — D10] ──── S3003 signal (orange)   ← spool drive (1:1 gearbox)
[Pin 5 — D11] ──── SG90  signal (orange)   ← drum metering (servo needs a digital pin)

[Pin 6 — A3]  ──── 100Ω ── Solenoid #1 IRLZ44N Gate   ← punch #1 (on/off, analog pin OK)
[Pin 7 — D13] ──── 100Ω ── Solenoid #2 IRLZ44N Gate   ← punch #2 (fit later)

[Pin 3 — 12V] ──── 1N4007 cathode ──── Solenoid(s) (+)
                          │
                     1N4007 anode ── Solenoid IRLZ44N Drain ── Solenoid (−)
```

**Servo control (both):** standard 50Hz PWM. The **S3003** (180°) swings the spool
via the 1:1 gearbox; the **SG90** rotates the drum to fill (pockets up) then drop
(pockets down). No stepper, no ULN2003 — tune fill/drop angles in firmware.

---

## ESP32-CAM

No data wires — communicates over WiFi only.

```
ESP32-CAM 5V  → UNO Q 5V rail (in circuit box)
ESP32-CAM GND → GND
(WiFi connection to UNO Q hotspot "FarmOS-AP")
```

Stream URL: `http://<esp32-ip>:81/stream`

Add 100µF capacitor across 5V + GND at the ESP32-CAM if it resets during heavy WiFi TX.

Mount the camera on robot front, angled 30° downward at ~25cm height.

---

## Audio

> The UNO Q has **no onboard audio output** (no codec on the header/USB-C audio path).
> Linux-side audio comes from a **USB Audio Class device** (USB sound card). See
> `plan.md → E1`.

```
UNO Q USB ──► USB sound card (CM108, class-compliant) ──► PAM8403 amp ──► 8Ω speaker
                                              5.0V (XY-3606) ──┘
```

- **Output (voice/TTS):** USB sound card headphone-out → **PAM8403** (~1.5W into 8Ω,
  fine for alerts) → salvaged **8Ω speaker**. Power the amp from the **5.0V** rail
  (never >5.5V); 470–1000µF across its 5V + keep audio-in on shielded wire away from
  motor leads to avoid whine.
- **Input (mic):** done from the **phone's mic via the PWA** over WiFi — no on-robot
  mic needed, so the sound card's mic input is unused (a plain output DAC also works).
- **Beeps/alarms only?** Skip the sound card + amp — the MCU's `tone()` drives the 8Ω
  speaker directly through a ~33–100Ω series resistor (already bench-validated).

---

## UNO Q Pin Summary

| Pin | Signal | Connected to |
|---|---|---|
| D2 | DS18B20 data | Temp sensor (1-Wire) + 4.7kΩ pull-up |
| D3 | IBT-2 #1 RPWM | Left forward PWM |
| D4 | IBT-2 #2 R_EN | Right driver enable |
| D5 | IBT-2 #1 LPWM | Left reverse PWM |
| D6 | IBT-2 #2 RPWM | Right forward PWM |
| D7 | IBT-2 #1 R_EN | Left driver enable |
| D8 | IBT-2 #1 L_EN | Left driver enable |
| D9 | IBT-2 #2 LPWM | Right reverse PWM |
| D10 | S3003 signal (PWM) | Seeder spool drive (via cable pin 4) |
| D11 | SG90 signal (PWM) | Seeder drum metering (via cable pin 5) — servo needs a digital pin |
| D12 | IBT-2 #2 L_EN | Right driver enable |
| D13 | Free (was solenoid #2) | Freed by the single-driver decision; `farm_os.ino` uses **D13 as the EC-drive** pin (placeholder — move to a JMISC pin for a production soil probe). Seeder/soil are mutually exclusive, so no clash. |
| A0 | Moisture #1 | Front-left soil probe |
| A1 | Moisture #2 | Front-right soil probe |
| A2 | EC sense | DIY EC probe midpoint (22kΩ pulldown) |
| A3 | Solenoid MOSFET gate (via 100Ω) | Fires **BOTH** punches — single driver (on/off; analog pin OK; via cable pin 6) |
| A4 | — (spare) | Soil-probe attachment analog (own connector) |
| A5 | — (spare) | Soil-probe attachment analog (own connector) |
| 5V | Power out | XY-3606 → 5V in, distributes to all components |
| GND | Common | All grounds |
| **D0/D1** | **RESERVED** | **RouterBridge — do not connect** |
| JDIGITAL Serial1 | BN-880 UART | GPS TX/RX |
| JMISC GPIO 1 | EC drive (AC) — target | Toggled square wave → DIY EC probe. *(Firmware placeholder is **D13** today; move here for production.)* |
| JMISC GPIO 2–4 | — (free) | Was stepper — freed by v2; spare for sensors |
| I2C SDA/SCL | BN-880 compass | HMC5883L heading |
| USB | USB sound card | Audio out (→ PAM8403 → 8Ω speaker) |

---

## Power Protection — fuses and wire gauge

> **A fuse protects the thinnest WIRE downstream of it, not the load.**
> This is the single rule. Everything below follows from it.

**Learned by setting a wire on fire, 2026-08-15.** The 12V solenoid feed was a Dupont
jumper. Moving the battery inside the robot shifted the wiring, the 12V feed shorted,
and the jumper **burnt through while the 20A main fuse never even warmed up**. Nothing
was wrong with the circuit — the solenoid had been firing correctly for days, the diode
was the right way round, the MOSFET was fine. The wire was simply 13× weaker than its
own fuse, so *the wire became the fuse*.

### Every power branch needs its own fuse

One 20A main fuse cannot protect a branch wired in anything thinner than 20A-rated cable.
It only protects the heavy distribution wiring between the pack and the block.

| Branch | Running current | Fuse | Wire |
|---|---|---|---|
| **Main** (LiPo+ → distribution block) | motors dominate | **20A** (yellow blade, fitted) | heavy / XT60 |
| **Motors** (block → IBT-2 ×2) | ~2–8A, stall higher | covered by main | heavy / IBT-2 screws |
| **Solenoid** (block → driver board 12V) | **0.75A** (1.5A with both punches) | **2A** (3A once punch #2 is fitted) | **1mm² stranded** |
| **Buck input** (block → buck 12V in) | ~1.5A peak (board + 2 servos) | **3A** | **1mm² stranded** |

Fuse for the **fault** current; gauge for the **running** current. They are different jobs
and you need both.

### Wire gauge reference

| Wire | Cross-section | Use |
|---|---|---|
| Dupont jumper | 26–28 AWG ≈ **0.08–0.13 mm²** | ⛔ **NEVER on a power branch.** Signal only. |
| 20 AWG silicone | 0.52 mm² | minimum for any power branch |
| **1mm² stranded** ≈ 17 AWG | **1.0 mm²** | ✅ what the solenoid + buck branches now use |

**Stranded, not solid.** Solid core work-hardens and cracks where it flexes, and is stiff
enough to lever a solder pad or terminal screw loose — on a robot that vibrates and gets
carried around, that is a fault waiting to happen. Strain-relieve both ends so the joint
never takes mechanical load, and route power away from the battery and anything that moves.

> The trigger here was **movement**, not electronics. Secure the LiPo so it cannot shift:
> a pack free to slide is free to chafe through insulation. See `troubleshooting.md` →
> `[Power] 12V solenoid feed caught fire`.

## Robust Connectors — stop the intermittent disconnects

Loose individual Dupont pins are the #1 cause of field disconnects (back out, oxidize, wiggle loose). The whole connector strategy reduces to **three rules by wire type:**

```
  Jumper / signal wire?  → CHOCO BLOCK (screw connector strip)
  Thick power wire?      → POWER BLOCK (barrier block / XT60 / IBT-2 screws)
  Into the UNO Q pins?   → GANGED + GLUED housing (one fixed joint)
```

**1. Jumper wire → choco block.** Anywhere a jumper/Dupont wire runs today, screw its loose end into a **choco block / connector strip** (5A, cut to length) instead of mating Dupont-to-Dupont. Each cell joins two wires through an internal brass barrel (no PCB pins, no solder). Covers *all* UNO Q pin connections — they're all signal/low-current. Use **two strips: one in the circuit box, one in the seeder box** (the 10-pin seeder cable links them). Fold thin wires over (or use fork terminals) so the screw clamps solidly.

**2. Thick power wire → power block.** Battery/motor current (LiPo 12V → IBT-2 → motors) **never comes from the UNO Q** and must NOT go through choco/signal blocks. Keep it on the **heavy barrier/distribution block + XT60 + IBT-2 screw terminals**, with the inline fuse on the LiPo+ before the block. ⚠️ That main fuse protects **only** the heavy wiring — every thinner branch off the block needs its own inline fuse sized to *its* wire. See **Power Protection** above; getting this wrong set a wire on fire.

**3. Into the UNO Q pins → ganged + glued.** The choco joins *wires*, but a wire still has to plug into the UNO Q's female socket — that joint can't be screwed. Make it robust: crimp the header pins into **one multi-pin housing** (or use a ribbon cable with a single housing), seat it, then **hot-glue the housing to the header shroud** so it physically can't back out. Peels off later → UNO Q stays reusable. For **JMISC / JDIGITAL / I2C**, use the board's **locking plug** if those ports are JST/Grove type — they latch and need no glue. This leaves exactly ONE fixed joint at the board; everything downstream is screw-terminated.

> **Why not a screw-terminal shield or PCB-pin terminal block?**
> - A UNO-footprint **screw shield** only breaks out the main header (D0-D13/A0-A5) — it **can't reach the UNO Q's JMISC/JDIGITAL/I2C**, so it misses the GPS and compass, and still needs tinkering to fit the UNO Q.
> - **PCB-mount terminal blocks** (5.08mm THT kit) have solder legs and screws that *don't* interconnect — gluing them together connects nothing; they need a protoboard + soldering. The choco block (internally-linked screws, no pins) does the job standalone with no solder.

## MOSFET Driver Modules — off the breadboard, onto stripboard

The solenoid MOSFET circuits are **permanent** — never reused elsewhere — so the "keep it solderless for reuse" rule (which protects the UNO Q) does **not** apply here. Move them off the flaky breadboard onto **stripboard (Veroboard)**: fewer joints = fewer failure points.

**Up to two modules** (both in the seeder box; build #2 only for dual-punch):

| Module | Box | Switches | Load V | Signal pin |
|---|---|---|---|---|
| Solenoid #1 driver | Seeder box | JF-0530B solenoid | 12V | D11 (via cable pin 6) |
| Solenoid #2 driver | Seeder box | JF-0530B solenoid #2 | 12V | D13 (via cable pin 7) |

**BOM per module (identical 4 parts):**
- IRLZ44N MOSFET (logic-level)
- 100Ω gate resistor
- 10kΩ gate→source pulldown (holds gate low during boot/float)
- 1N4007 flyback diode across the load (**band → +V side**)

**Stripboard layout** (cut a ~3×4 cm piece per module from a 10×8 cm Veroboard):
- Orient the IRLZ44N so its **3 legs land on 3 separate strips** → Gate / Drain / Source stay isolated automatically.
- Each strip = one node:
  - **Gate strip:** 100Ω (to signal-in) + 10kΩ (to source strip)
  - **Drain strip:** load(−) wire + 1N4007 anode
  - **Source strip:** GND wire + 10kΩ other end
- The load(+) wire goes straight to the 12V/5V rail and to the 1N4007 cathode (does not need to touch the MOSFET strips).
- **Cut any strip** that would otherwise connect two of these nodes (twist a 3mm drill bit by hand in the hole, or nick with a knife).

**4 wires leave each board** → land on the choco block:
```
  IN  → signal (D11 / D13)        [→ 100Ω → Gate]
  GND → common ground             [→ Source]
 LOAD+→ +12V (solenoid)           [→ solenoid + and 1N4007 cathode]
 LOAD−→ solenoid negative         [→ Drain, 1N4007 anode]
```

> **Before power:** continuity-test the three nodes — Gate, Drain, Source must each be isolated (no strip accidentally bridging them). This catches a missed track-cut, the one stripboard gotcha.

**No-solder fallback:** a pre-made logic-level MOSFET module with screw terminals — but verify it uses a logic-level FET (not IRF520) and **add a 1N4007 flyback across the coil externally** (most modules omit it).

## First-Power Checklist

1. Measure XY-3606 output with multimeter — must be 5.0V ±0.1V before connecting to UNO Q.
2. Confirm D0/D1 have nothing connected.
3. Seeder box cable disconnected for first boot — test motors alone first.
4. All grounds in circuit box form one continuous chain (LiPo → IBT-2 → XY-3606 → UNO Q GND).
5. SSH into UNO Q, open serial monitor, confirm STM32 sketch is running before testing motors.
6. Reconnect seeder cable only after `test_motors.py` passes.
7. Test seeder box separately with `test_seeder.py` after reconnecting cable.
