# Farm OS — Troubleshooting Log

---

## [App Lab] Model installs but the runner still serves `yolo-x`; app vanishes from App Lab (2026-08-16)

Installing the trained FOMO model took most of a day. Recording the whole shape of it,
because almost none of this is in the docs and every failure mode was **silent**.

### The model is bound in `app.yaml`, not by the UI

Brick entries are **mappings**, and `model:` is the binding:

```yaml
bricks:
- arduino:web_ui: {}
- arduino:object_detection:
    model: ei-model-1088852-1     # id from: curl 127.0.0.1:8800/v1/models
```

App Lab then resolves it to its own store and writes the real path into the generated
compose (`.cache/app-compose-overrides.yaml`):

```
EI_OBJ_DETECTION_MODEL: /home/arduino/.arduino-bricks/models/custom-ei/<id>/model.eim
```

**Because it lives in `app.yaml` it survives `arduino-app-cli app start` and a reboot.**
Before we found this, every restart silently reverted to the stock `/models/ootb/ei/
yolo-x-nano.eim` — the app ran, the runner reported *healthy*, and it served 80 COCO
classes. Nothing anywhere said the wrong model was loaded. **Always verify:**

```bash
ssh unoq 'curl -s http://127.0.0.1:1337/api/info' | head -5    # want labels ['emitter']
```

### Dead ends that cost hours — don't repeat them

| Attempted | Why it fails |
|---|---|
| Copy the `.eim` into `~/.arduino-bricks/ei-models/` | the runner's `--model-file` comes from compose interpolation, not from that directory |
| Export `EI_OBJ_DETECTION_MODEL` before `app start` | `arduino-app-cli` does not pass its shell environment to compose |
| A `.env` in the compose project dir (`.cache/`) | not read |
| Hand-edit `app-compose-overrides.yaml` | **regenerated on every app start** — silently reverted |
| Register a custom model via the API | `/v1/models` is **read-only** (`Allow: GET, HEAD`) and cloud-backed |

### The apps directory moved

The Aug-2026 App Lab runtime upgrade (offered as a "library update") changed:

```
Apps Directory: /home/arduino/ArduinoApps      ← new; App Lab ONLY scans here
```

Our app at `~/motor-control` **disappeared from App Lab while still running**. Nothing was
lost — it just wasn't being looked for. `arduino-app-cli config get` shows the directory.

Fix: move the app into `~/ArduinoApps/` and leave `~/motor-control` as a **symlink** so
existing deploy paths keep working. Do NOT "import from computer" — that creates a second
copy that diverges from the one the deploy scripts write to.

> ⚠️ Starting the app **by path** while it is also discovered via `ArduinoApps/` produces
> **two registrations** (`./motor-control` and `user:motor-control`) for one directory.
> App Lab then greys out **Run**, because its own entry is idle while the containers are
> taken by the other. Keep exactly one.

### App Lab needs the USB cable — but adb is fragile

App Lab reaches the board over **USB**, not the network: its daemon binds `127.0.0.1:8800`
only, and `arduino-router-serial` proxies to `ttyGS0`. Over the network you get the brick
catalogue and the cloud Edge Impulse link (so your project appears) but **no apps**, and
Download fails with nothing in the board's logs.

⚠️ **Do not run any other `adb` while App Lab is connected.** Only one adb server can own
the device. We ran a `adb devices` check mid-session and the device went to `offline` and
stayed there — through a server restart, a cable replug, an `adbd` restart, *and* a full
board reboot, for both adb 32.0.0 and 37.0.0. Root cause never confirmed; avoid the
situation rather than trying to recover it.

### Also worth knowing

- The env var gating our ML path (`FARMOS_EMITTER_ML`) now **defaults ON**. App Lab
  regenerates both compose files on every start, so there is nowhere durable to inject an
  env var into the app container — a flag that cannot be set is a flag that is always off.
  `FARMOS_EMITTER_ML=0` still forces the classical fallback.
- Switching the board's WiFi mid-session changes its IP and **App Lab keeps the old one** —
  the board then appears in the list but clicking it does nothing. Restart App Lab.

---

## [Gyro/I2C] The header pins labelled SDA/SCL have NO I2C — the gyro lives on A4/A5 (2026-08-17)

Fitting an MPU-6050 to close the loop on turns took a whole afternoon and six MCU
reflashes, almost entirely because **the pins silk-screened `SDA`/`SCL` on the UNO Q
header do not have an I2C peripheral behind them.** Recording the whole chain, because
every wrong turn looked exactly like a wiring fault.

### Why not D20/D21 (the pins actually labelled SDA/SCL)

The core's own board overlay hands those pads to USART3 and leaves `i2c2` with no
`status` and no `pinctrl`:

```
&usart3 { status = "okay"; pinctrl-0 = <&usart3_tx_pb10 &usart3_rx_pb11>; };
&i2c2   { zephyr,deferred-init; };      <- no status, no pinctrl: DISABLED
```
(`~/.arduino15/packages/arduino/hardware/zephyr/<ver>/variants/arduino_uno_q_stm32u585xx/
arduino_uno_q_stm32u585xx.overlay`)

So `Wire.begin()` succeeds, every transfer fails, and a bus scan finds nothing however
correctly the module is wired. Matches [ArduinoCore-zephyr#301](https://github.com/arduino/ArduinoCore-zephyr/issues/301).

**This cannot be patched at the bench.** The overlay is compiled into the prebuilt
`zephyr-*.elf` that gets flashed, so editing the text does nothing without rebuilding
Zephyr from source.

**The pads still work as plain GPIO** — `gateTest(20)`/`gateTest(21)` return
`dr_hi:1, dr_lo:0` and look perfectly healthy. That is a trap: it proves the pin drives,
not that a peripheral is attached.

### The mapping — NOT what the Arduino forum says

From the overlay's `i2cs = <&i2c2>, <&i2c4>, <&i2c3>;`:

| object  | peripheral | pins                     | usable |
|---------|-----------|--------------------------|--------|
| `Wire`  | i2c2 | D20/D21 (labelled SDA/SCL) | **NO** — disabled |
| `Wire1` | i2c4 | Qwiic socket only          | yes, needs a JST-SH cable |
| `Wire2` | i2c3 | **A4 / A5**                | yes, and solderable |

A forum post claimed header = `Wire` and Qwiic = `Wire1`. It is wrong in both halves.
**Read the overlay on the board, not the forum.**

### So the gyro is on A4 (SDA) / A5 (SCL) via `Wire2`, and it cost three things

1. **The battery divider lost A4** — `BATT_PRESENT 0`. `getBattery` now answers
   `present:false` instead of reporting a floating pin, because a plausible-looking wrong
   voltage is worse than none. Acceptable trade: the divider was never stable enough to
   compensate duty with (the console has always run `batt_comp=False` and field runs pass
   `nobatt`), whereas the turn had no other way to know what it actually did.
   To reverse it: `BATT_PRESENT 1`, refit the 10k/2k + 100nF to A4, and move the gyro to
   the Qwiic connector (`Wire1`) — which needs a cable.
2. **`gateTest` must not `analogRead` A5** — `GATE_SENSE 0`. `analogRead()` RE-MUXES the
   pad before sampling (see the gateTest entry below), so one call would take SCL away
   from i2c3 mid-run.
3. **`getDiag` had to stop reading A4.** This one cost the most time: `rpc_getDiag()`
   called `readBattRaw()` unconditionally, and the console's battery logger polls
   `getDiag` **every 2 seconds** — so the bus died again a moment after every
   `Wire2.begin()`. Symptom: a correctly-wired sensor that scans clean exactly zero
   times. Both call sites are now `#if BATT_PRESENT`.

### Power it from 5V, not 3V3

The GY-521 feeds VCC into an **onboard 3.3V regulator**, which needs headroom. Powered
from the UNO Q's `+3V3`, the module's rail — and therefore its pull-ups — measured
**2.14V**. The STM32's guaranteed logic-high threshold is ~0.7 x 3.3 = **2.31V**, so
idle-high never registered as high and I2C could not start. The vendor spec confirms
`Input power supply 5V`. UNO Q header pins are 5V tolerant (only A0/A1 are not).

### Diagnosing a dead I2C bus without a meter: `i2cLines`

Reads each pad floating, then with the MCU's internal pull-up. The **pair** identifies the
fault; either reading alone is ambiguous:

| float | pull-up | meaning |
|-------|---------|---------|
| 1 | 1 | external pull-up present — the module is connected |
| 0 | 1 | no external pull-up: that wire is not reaching the module's pin, **or** the rail is below V_IH (the 2.14V case) |
| 0 | 0 | pad held down — short to GND, or something else still on the pin |

Write this before theorising. It would have saved three of the six reflashes.

### The module is not really an MPU-6050

`WHO_AM_I` reads **0x74**. A genuine MPU-6050 returns **0x68**, because that register
contains the device's own I2C address bits (`110100` for AD0=0) — so 0x74 at address 0x68
is self-inconsistent. The vendor page for this board says outright: *"This board will
function as MPU6500."* Consequences:

- Gyro registers and the sensitivity scale factors (131 / 65.5 / 32.8 / 16.4 LSB per
  deg/s for FS_SEL 0..3) are the same, so the code works.
- **`GYRO_CONFIG[1:0]` is `FCHOICE_B` on the 6500** (reserved on a real 6050). Non-zero
  BYPASSES the DLPF and runs the gyro at 8/32kHz, undoing the anti-aliasing. Keep them 0
  — `0x08` does.
- The 6500 is **more** vibration-sensitive than the 6050 (vendor's own comparison), so the
  on-chip DLPF matters more, not less.
- Never assume the sensitivity: `imuBegin()` reads `GYRO_CONFIG` back and scales from the
  FS_SEL the chip reports. `gspin` measures the true scale against a hand-turned 360.

### Vibration settings that matter (any IMU on this robot)

- **+/-500 dps** full scale, not the +/-250 default: a vibration spike that CLIPS becomes
  asymmetric noise and integrates into real error, while unclipped vibration averages out.
- **DLPF_CFG = 3** (44Hz): filtered ON-CHIP, i.e. before sampling. Without it, motor and
  gearbox vibration aliases down into what looks like genuine slow rotation.
- **Re-sample bias standing still immediately before every pivot.** Bias drifts with
  temperature and cannot be measured while the motors run.

---

## [Power] 12V solenoid feed CAUGHT FIRE — 20A fuse on a Dupont jumper (2026-08-15)

**What happened:** the 12V feed from the terminal block to the solenoid driver board was a
**Dupont jumper**. While moving the battery and wires inside the robot, that feed shorted.
The jumper burnt through. **The 20A main fuse never blew.**

**Root cause — not electrical, mechanical + a protection mismatch:**

| | Current it passes |
|---|---|
| Main fuse (yellow blade) | **20 A** |
| Dupont jumper, 26–28 AWG | **~1.5–2 A** before it melts |

A **13× mismatch**, so the wire was the weakest link in its own circuit: *the wire became
the fuse*. The trigger was **movement** — a pack free to shift chafed/shorted the feed.

**Ruled out, and why (all were healthy):**
- **Diode orientation** — a reversed 1N4007 shorts on the *first* power-up. This solenoid
  had been firing correctly for days, so the diode was right. Working history is proof.
- **MOSFET, fuse, coil, terminal block** — all fine afterwards; the robot still drove and
  ran the seed flow with the solenoid disconnected.

**Fix:**
1. **1mm² stranded** (≈17 AWG, ~2× the copper of the 20 AWG spec) for the 12V feed, soldered.
2. **Inline 2A fuse on the solenoid branch** (3A once punch #2 is fitted). ← the important one.
3. Strain-relieve both ends; route clear of the battery; **secure the LiPo so it can't shift**.

**The general lesson, which applies to every branch:** with a single 20A fuse, *any* short
anywhere burns a wire instead of blowing the fuse, unless every conductor downstream is
rated above 20A — and none of them are. **Fuse for the fault current, gauge for the running
current.** The fused branches are drawn on sheet 1 of `docs/schematic/`.

> Upgrading the wire alone is NOT the fix. A 3S LiPo will push 50–100A into a dead short,
> and 1mm² still gets destructively hot at that — it just takes longer than the Dupont did.
> The branch fuse is what actually stops the next fault.

---

## [Seeder] Solenoid never fired — it was on the 5V rail, not 12V (2026-08-14)

**Symptom:** `punch` / `plantSeed` did nothing. Both servos worked fine. It *had* worked
weeks earlier, so nothing was inherently wrong with the design — something had changed.

**Two dead ends worth recording, because both cost time:**
1. A stale wiring diagram said the MOSFET gate was on **D11**. The firmware drives **A3**.
   That looked like a smoking gun (D11 is now the SG90's pin) — but the physical IN wire
   really was on A3, and only the diagram was out of date. *Trace the wire before
   theorising from a diagram.*
2. Suspicion that A3 can't drive a GPIO output at all, since the pinout already
   records "SG90 was dead on A3". **A3 is fine for digital output** — it fails only for
   *servo* PWM (no timer channel). The overlay confirms A3=PA7 is in `digital-pin-gpios`.

**How it was actually found — measure, in this order:**

| # | Probe | Expect | Reading we got |
|---|---|---|---|
| 1 | +12V (solenoid +) → GND | ~12V | **5.45V**  ← the fault |
| 2 | Gate → Source, asserted | ~3.3V | OK |
| 3 | Drain → GND | 0V fired / supply at rest | 0V fired / 1.48V rest |

**Root cause:** the solenoid `+` was landed on the **buck's 5V output** instead of the
fused 12V LiPo feed (**inter-box cable pin 3**). Almost certainly relanded during the
buck removal/refit. The JF-0530B is a **12V, 5N** part, and solenoid force goes as V²:

    (5.45 / 12)² ≈ 0.21  →  ~1.0N of the rated 5N; the tip needs 2–4N.

So it could not move, even though every part of the control path was healthy.

**Fix:** move the `+12V` node (solenoid `+` **and** the 1N4007 cathode — same node) to
the fused 12V feed. Verify the diode band still faces +12V.

**Lesson:** a working gate and a switching MOSFET tell you nothing about the supply.
Measure the rail first — it's the cheapest probe and it was the answer.

> Also note: a ~0.75A inductive load on the UNO Q's own 5V rail is a brownout source.
> Even had it worked, it belonged on 12V.

---

## [Seeder] `gateTest` on a servo pin HANGS the MCU — Servo detach()/attach() (2026-08-14)

**Symptom:** after calling a diagnostic RPC that did `spoolServo.detach()` → test the pin
→ `attach()`, **every** subsequent RPC timed out, `getBattery` included. The bridge was
dead until the MCU was reflashed.

**Root cause:** the Zephyr core's `Servo` library owns a hardware counter on its pin and
does **not** survive being detached and re-attached at runtime. It wedges the MCU.

**Fix:** `rpc_gateTest()` now **refuses** `SPOOL_PIN` and `DRUM_PIN` outright. Diagnose a
servo through the Servo API (`indexSpool` / `dropSeed`), never by re-muxing its pin.

**Recovery:** reflash the MCU (openocd — see the MCU-flash recipe in the ai-labs repo's
`apps/farm-robot/CLAUDE.md`, recipe B), then
`arduino-app-cli app start`. `app restart` alone does NOT reset a wedged MCU.

> Related dead end: `gateTest` also reported `analogRead()` of the *driven* pin. That
> value is meaningless — `analogRead()` re-muxes the pad to analog mode *before*
> sampling, so it measures the pin after the output driver is already disconnected.
> Three pins in identical states read 1261 / 2109 / 1631. Removed. Use `digitalRead()`,
> which on STM32 reads the input data register and so reflects the real pad level.

---

## [Stage 1] SSH setup on UNO Q Linux

**Problem:** `systemctl start sshd` fails with "Unit sshd.service not found." The correct service name is `ssh` (not `sshd`).

**SSH host keys missing:** `sshd: no hostkeys available`. Generate them with `sudo ssh-keygen -A`.

**Password auth blocked:** Default sshd config has `KbdInteractiveAuthentication no`, which blocks PAM password login. Fix with key-based auth — copy public key via ADB:
```bash
# On Mac:
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519_unoq -N ""
PUB=$(cat ~/.ssh/id_ed25519_unoq.pub)
adb shell "mkdir -p ~/.ssh && echo '$PUB' >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
# Then SSH:
ssh -i ~/.ssh/id_ed25519_unoq arduino@<IP>
```

**WiFi:** Connect via `nmcli device wifi connect 'SSID' password 'PASS'`. Check IP with `ip addr show wlan0`.

---

## [Stage 1] arduino-app-cli app structure and Bridge.call() API

**Problem:** Running Python scripts that use the RouterBridge requires a specific app directory structure and Docker environment provided by arduino-app-cli.

**App directory structure:**
```
my-app/
  app.yaml          # name, icon, description
  python/main.py    # your Python script
  sketch/sketch.yaml  # fqbn + libraries (copy from any example)
```

**Run with:** `arduino-app-cli app start /path/to/my-app`

**Check logs:** `arduino-app-cli app logs my-app`

**Bridge.call() returns the value directly** — no `.result()` needed:
```python
from arduino.app_utils import Bridge, App

pong = Bridge.call("ping")          # returns "pong" directly
Bridge.call("setMotors", 150, 150)  # returns 1
state = Bridge.call("getMotorState") # returns JSON string
```

`App.stop()` does not exist — the app exits when `loop()` returns normally.

---

Issues encountered during build, with root cause and fix.
Add new entries at the top (newest first).

---

## [Stage 1] arduino-cli sketchbook path not configurable via `config set`

**Error:**
```
Error setting value: key not found: sketchbook.path
```

**Root cause:** `arduino-cli config set sketchbook.path` is not a valid key in arduino-cli 1.x. The correct key lives under `directories.user` in the YAML config file.

**Fix:** Edit `~/Library/Arduino15/arduino-cli.yaml` directly:
```yaml
board_manager:
    additional_urls: []
directories:
    data: /Users/saravananshanmugam/Library/Arduino15
    downloads: /Users/saravananshanmugam/Library/Arduino15/staging
    user: /Users/saravananshanmugam/arduino-sketchbook
```
Then create the directory: `mkdir -p ~/arduino-sketchbook/libraries`

---

## [Stage 1] arduino-cli cannot access libraries installed by Arduino IDE

**Error:**
```
ls: /Users/saravananshanmugam/Documents/Arduino/libraries/: Operation not permitted
Error installing Arduino_RouterBridge: destination dir .../Arduino_RouterBridge already exists, cannot install
```

**Root cause:** macOS restricts terminal (and sandboxed tools like Claude Code) from accessing `~/Documents` unless explicitly granted permission in System Settings → Privacy → Files and Folders. Arduino IDE has that permission; the terminal does not.

**Fix:** Configure arduino-cli to use a different sketchbook path outside `~/Documents` (see entry above). Then install libraries via arduino-cli instead of the IDE:
```bash
arduino-cli lib install Arduino_RouterBridge
```
Libraries land in `~/arduino-sketchbook/libraries/` which is fully accessible.

---

## [Stage 1] `Bridge.bind()` does not exist — correct method is `Bridge.provide()`

**Error:**
```
error: 'class BridgeClass' has no member named 'bind'
```

**Root cause:** The `Arduino_RouterBridge` library (v0.4.2) does not expose a `.bind()` method. The correct method to register an RPC function is `.provide()`. The internal `server->bind()` call exists inside the library but is not part of the public API.

**Fix:** Replace all `Bridge.bind(...)` with `Bridge.provide(...)` in the sketch:
```cpp
// Wrong
Bridge.bind("ping", rpc_ping);

// Correct
Bridge.provide("ping", rpc_ping);
```

---

## [Stage 1] Wrong include for RouterBridge — `RPC.h` does not exist

**Error:**
```
fatal error: RPC.h: No such file or directory
```

**Root cause:** The UNO Q RouterBridge library is not included as `RPC.h`. The correct header is `Arduino_RouterBridge.h`. The board package ships a stub header that gives this error message deliberately to force users to install the real library.

**Fix:**
1. Install library: Arduino IDE → Manage Libraries → search `Arduino_RouterBridge` → Install  
   Or via CLI: `arduino-cli lib install Arduino_RouterBridge`
2. Change include in sketch:
```cpp
// Wrong
#include <RPC.h>

// Correct
#include <Arduino_RouterBridge.h>
```

---

## [Stage 1] Upload verify errors on STM32U5 dual-bank flash

**Error (during upload):**
```
Error: verify failed in bank at 0x08000000 starting at 0x00000000
Error: verify failed in bank at 0x08000000 starting at 0x00100000
```

**Root cause:** The STM32U585 uses dual-bank flash (2 × 1MB). OpenOCD verifies both banks after write. The sketch only occupies one bank (~10%), so the second bank verify reports a mismatch. This is a known OpenOCD/STM32U5 behaviour — the upload completes correctly.

**How to confirm upload succeeded:** After upload, the board reboots and reappears on the same USB port:
```
New upload port: /dev/cu.usbmodem16606677722 (serial)
```
Run `arduino-cli board list` — if it shows "Arduino UNO Q", the firmware is running.

**Fix:** No action needed. Treat these verify errors as warnings, not failures.

---

## [Stage 1] Arduino UNO Q not appearing as USB serial device

**Symptom:** `ls /dev/tty.usbmodem*` returns nothing. `system_profiler SPUSBDataType` shows no Arduino.

**Root cause:** USB-C cable was charge-only (no data lines). The UNO Q requires a full USB-C data cable to enumerate as a serial device.

**Fix:** Replace the USB-C cable with one known to carry data (e.g. a cable used for phone data transfer or the one supplied with the board). After replug, it should appear as:
```
/dev/cu.usbmodem<serial_number>
```
Vendor ID will be `0x2341` (Arduino LLC).

---

## [Stage 1] arduino-cli library index not found / lib install fails silently

**Symptom:** `arduino-cli lib install ...` fails or gives no output.

**Root cause:** arduino-cli needs its library index downloaded before installing. Fresh installs have an empty index.

**Fix:** Run index update before any lib install:
```bash
arduino-cli core update-index
arduino-cli lib update-index
arduino-cli lib install Arduino_RouterBridge
```

---

## General — arduino-cli one-shot compile + upload command

For reference, the full command to compile and flash in one step:
```bash
arduino-cli compile --upload \
  --fqbn arduino:zephyr:unoq \
  --port /dev/cu.usbmodem16606677722 \
  firmware/farm_os/farm_os.ino
```

Useful flags:
- `--verbose` — show full compiler output
- `--verify` — verify after upload (skip if you see dual-bank false errors)

---

## General — find connected Arduino port

```bash
arduino-cli board list
# or
ls /dev/cu.usbmodem*
```

---

## [Network] Cannot reach the port-7000 UI or SSH ("worked then stopped")

**Problem:** The Farm OS UI at `http://<robot-ip>:7000` (and SSH) becomes unreachable, often after previously working. Symptoms: ping times out, ports 7000/22 time out.

**Root causes found (in likelihood order):**
1. **Robot's DHCP IP changed.** The router hands a new IP on reboot (seen at `.91`, `.204`, `.110`). A bookmark or `~/.ssh/config` pointing at an old IP silently breaks. → the #1 cause of "worked then stopped."
2. **The test computer wasn't actually on the same WiFi.** A Mac showed `en0` with IP `.48` but `networksetup -getairportnetwork` reported "not associated," and `arp <robot-ip>` returned `(incomplete)` — no Layer-2 path. It was on a different/Ethernet/guest network, not home WiFi WiFi. The **phone on home WiFi WiFi reached the robot fine.** Lesson: test from the device that matters and confirm it's truly on the same SSID before blaming the robot.
3. **mDNS name didn't resolve** — `farm-os.local` failed because `avahi-daemon` was installed but **disabled**.

**Fixes:**
- **Stable name via mDNS (primary, DONE):** enable avahi so the robot always answers to `farm-os.local` no matter what IP DHCP gives:
  ```bash
  sudo systemctl enable --now avahi-daemon   # on the robot
  ```
  Then use **`http://farm-os.local:7000`** everywhere — survives IP changes. ✅ Confirmed working from phone.
- **Backup — DHCP reservation:** in the home WiFi router admin (`http://192.168.31.1`), bind the robot's `wlan0` MAC to a fixed IP.

> The board + server were healthy the whole time (SSH on `:22`, server `python /app/python/main.py` in Docker on `0.0.0.0:7000`). This was purely an addressing/reachability problem — a reboot does NOT fix it.

---

## [Network] Diagnosing the robot when it's unreachable over WiFi (use USB/ADB)

When the network path is down, reach the robot over the USB cable and inspect from the inside:

```bash
adb devices                                   # board connected over USB?
adb shell "uptime"                            # did it actually reboot? (low = yes)
adb shell "ip addr show wlan0 | grep inet"    # current WiFi IP
adb shell "nmcli device status"               # WiFi connected? to which SSID?
adb shell "ss -tln | grep -E ':22 |:7000 '"   # are SSH + server listening?
adb shell "ping -c2 192.168.31.1"             # robot -> router (LAN ok?)
adb shell "ping -c2 8.8.8.8"                  # robot -> internet
```

Gotchas:
- `adb root` is **blocked** on this board — shell stays as user `arduino`. Privileged commands (`systemctl`, reboot) need `sudo` + password from a real shell.
- `adb reboot` returns "error: closed" and does **not** reboot — use `sudo reboot` from a shell.
- `adb forward tcp:7000 tcp:7000` -> `http://localhost:7000` works as a **dev-only** bypass over USB, but it is NOT a field solution.

---

## [Network] WiFi: one radio = hotspot OR home WiFi, not both

The robot has a single WiFi radio (`wlan0`) with two NetworkManager profiles:
- `<home-ssid>` — client/STA, `autoconnect=yes` (wins on boot)
- `FarmOS-AP` — hotspot/AP, IP `192.168.4.1`, `autoconnect=no`

It runs only one at a time. Usage:
- **Home (dev):** home WiFi for internet; reach the robot at `farm-os.local:7000`.
- **Field (no router):** activate `FarmOS-AP`; phone connects to it -> server at fixed **`http://192.168.4.1:7000`**.
- Need internet **and** phone hotspot simultaneously -> add a **2nd radio** (USB WiFi dongle).

---

## [Network] Quick field checklist (UI won't load)

1. Robot powered + booted? (power LED; wait ~60s after power-on)
2. Phone on the right WiFi — **FarmOS-AP** (field) or home WiFi (home), **not** cellular
3. Open **`http://farm-os.local:7000`** (home WiFi) or **`http://192.168.4.1:7000`** (FarmOS-AP)
4. Still dead? power-cycle the robot, wait ~60s, retry
5. Deeper: connect USB and run the ADB diagnostics above

**Self-healing status:**
- ✅ **Stable name** — `farm-os.local` via avahi (survives IP changes)
- ✅ **Crash recovery** — container restart policy `unless-stopped` (re-applied by `pnpm deploy:robot`)
- ✅ **Boot recovery** — `farm-robot-control.service` starts the app on boot (verified after reboot)
- ✅ **Network mode** — boot decides WiFi vs hotspot; manual switch from the UI (see below)
- ⬜ **TODO:** `/health` status page, boot audio announcing IP/status

**Deploy:** `pnpm deploy:robot` — auto-selects **SSH** (over WiFi, `unoq` alias) or **ADB** (USB); pushes app, restarts container, sets restart policy, waits for HTTP 200. One-time helper install: `pnpm deploy:helper`.

---

## [Network] WiFi-or-hotspot: boot decision + manual switch (one radio)

One WiFi radio → the robot is **either** on home WiFi (STA) **or** running the `FarmOS-AP` hotspot. Design is **boot-decision + manual** (no polling/auto-switch):

- **On boot** — `farmos-netmode.service` runs `/opt/farmos/net_mode.sh auto`: connects to home WiFi if in range (within ~24s), else starts the hotspot. So in the field (no home WiFi) it auto-falls-back to `FarmOS-AP` → phone connects to `192.168.4.1:7000`.
- **UI buttons** (relayed via the root helper → `net_mode.sh`):
  - **Connect WiFi** → switch to home WiFi; **2 retries, then auto-reverts to hotspot** (never strands itself). The hotspot drops on switch — reconnect the phone to home WiFi, open `yerbot.local:7000`.
  - **Hotspot** → force `FarmOS-AP`.

Files: `server/net_mode.sh`, `server/farmos-netmode.service`, actions added to `server/power_helper.py` (`/wifi`, `/hotspot`), UI in `apps/motor-control`. Logs: `journalctl -t yerbot-netmode`.

⚠️ **Test the hotspot path over USB only** — activating the hotspot drops home WiFi (and any SSH session). Over USB/ADB a WiFi flip can't lock you out.

---

## [Network] ESP32-CAM + always-reachable hotspot (decision 2026-08-05)

**ESP32-CAM is a CLIENT, never its own AP.** The UNO Q must *read* the cam's stream, so
both must sit on the **same network**. If the cam made its own hotspot the UNO Q couldn't
join it (single radio, busy as STA or FarmOS-AP) — the LAN would fragment. So the cam does
**not** do the UNO Q's connect-or-become-AP fallback; instead it **follows** the UNO Q onto
whichever network the UNO Q landed on, via a priority list (`WiFiMulti`) in the
CameraWebServer sketch:
```cpp
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
// in setup(), replacing the example's single WiFi.begin():
wifiMulti.addAP("FarmOS-AP",       "<hotspot-pw>");   // field — the robot's hotspot (try first)
wifiMulti.addAP("<home-ssid>", "<home-pw>");      // home/dev — same LAN as the UNO Q
while (wifiMulti.run() != WL_CONNECTED) delay(300);
```
Field (UNO Q = FarmOS-AP) → cam joins FarmOS-AP. Home (UNO Q = home WiFi) → cam joins
home WiFi. Either way it's on the UNO Q's network; stream at `http://<cam-ip>:81/stream`.

**"Always-on FarmOS-AP even while on home WiFi?"** Not reliable on the single radio —
concurrent AP+STA is chip/driver-dependent (see the one-radio note above; the sanctioned
"both at once" answer is a **2nd radio / USB WiFi dongle**). So:
- **Demo/field = force hotspot** (`net_mode.sh hotspot` or the UI button). No home WiFi in the
  field anyway → UNO Q + ESP32-CAM + phone all on FarmOS-AP, fully self-contained, no internet
  needed. This is the demo setup.
- **Internet + local devices together** → add a USB WiFi dongle (one radio on home WiFi for
  internet, one hosting FarmOS-AP).

---

## [Network] Graceful shutdown / reboot from the web UI

The web app runs in an **unprivileged container** and can't power off the host. A small **root helper** (`server/power_helper.py`) does it; the UI button → `main.py` → POSTs a token to the helper over the container gateway → `systemctl poweroff`/`reboot`.

**Files:** `server/power_helper.py`, `server/farmos-power-helper.service`, UI bits in `apps/motor-control` (`main.py`, `index.html`, `app.js`, `style.css`).

**One-time install on the robot (root):**
```bash
sudo mkdir -p /opt/farmos
sudo cp power_helper.py /opt/farmos/
sudo cp farmos-power-helper.service /etc/systemd/system/
sudo systemctl enable --now farmos-power-helper
```
Then re-deploy the `motor-control` app. The **Reboot** and **Shutdown** buttons (with confirm dialogs) appear under the speed slider. Token `farmos-power` is shared between `main.py` and the helper. Helper binds `0.0.0.0:7999` (token-protected); to restrict to the docker subnet, add an iptables rule.

---

## [Audio] Microphone input for speech recognition — research notes

**Question:** how to get mic input on the UNO Q for speech recognition?

**Findings (researched 2026-06-24):**
- **Silicon supports onboard mic** — device tree shows the pm4125 codec with `qcom,micbias1/2/3`, an `i2s2` interface (`lpi-i2s2-active-state`), a DMIC/VA macro (`va_macro … qcom,dmic-sample-rate dt entry missing`), and MBHC headset detection. `arecord -l` lists capture endpoints (MultiMedia1–4) — but **no physical mic is attached**.
- **Arduino documents ONLY the USB-C path** — the [UNO Q hardware page](https://docs.arduino.cc/hardware/uno-q) lists "USB microphone" via a USB-C dongle with power delivery. No I2S/mic header pinout is published. A [Jan 2026 forum user](https://forum.arduino.cc/t/how-to-connect-i2s-microphone-to-uno-q/1428067) hit the same wall (ICS43434 I2S mic, no wiring docs).
- **ANX7625** handles audio *out* over USB-C (separate from the pm4125 mic codec).

**Conclusion / plan:**
- **USB-C mic** works but consumes the single USB-C port — avoid.
- **I2S MEMS mic** (ICS-43434 / INMP441) on the headers is the right onboard, no-USB-C path — but **blocked on undocumented I2S pinout** (new board). Check the `ABX00162` datasheet PDF pinout table; watch the forum for Arduino to publish it. Then wire BCLK/WS/SD + power.
- **For now: phone-based speech** — browser SpeechRecognition (Tamil/English) on the phone → send recognized text as a command over the existing websocket. Zero robot hardware, works over the hotspot. The pragmatic, solved path.

---

## [Power] Shutdown won't stay off — board reboots ~1s after poweroff

**Symptom:** Tapping Shutdown in the UI (or `sudo poweroff` over SSH) halts the OS, but the board powers back on a second later. "Works sometimes."

**Root cause:** the board was being **powered through USB-C** (LiPo → buck → USB-C). On this Qualcomm board, **USB-C VBUS is a power-on signal** — after `poweroff` cleanly halts the OS, the PMIC still sees VBUS on USB-C and immediately boots again. The "sometimes" = whether USB-C was the power path at that moment.

**Proof from logs:** chain worked perfectly — `POST /shutdown → 200 → systemctl poweroff → Reached target System Shutdown` — then `journalctl --list-boots` shows a new boot **1 second later** (e.g. boot ended 14:42:35, next began 14:42:36). So it halts cleanly; it just won't *stay* off while VBUS is present.

**Note:** the UI button and SSH `sudo poweroff` are the **same command** (`systemctl poweroff`) — no behavioral difference; only the power conditions differ.

**Fix:** power the board via the **5V header pin** (XY-3606 → 5V/GND pins, set to exactly 5.0V, verify with multimeter), **not USB-C**. No VBUS → poweroff stays off. Bonus: frees the USB-C port. This is the wiring drawn on sheet 1 of `docs/schematic/`.

**Field workflow regardless:** the robot is powered by a LiPo through a switch — true "off" = **flip the switch**. The Shutdown button's job is the **clean OS halt first** (protects the eMMC) — tap Shutdown, wait ~15s for the halt, then cut power.

---

## [Camera] The ESP32-CAM was delivering 0.7-6.7 fps — replaced by a USB webcam (2026-08-18)

Three days of "course correction is broken" turned out to be the camera. Measured with the
robot app STOPPED and curl straight from a laptop, so no project code in the path:

    ESP32-CAM MJPEG:  11 frames in 8s = 1.4 fps, and variable 0.7-6.7 fps
    a single /capture: 2.75 s
    ping:              125 ms avg, 11 ms min, 300 ms peak

At 0.17 m/s that is 12-24 cm of ground per control measurement. Every steering gain, lookahead
weight and detector threshold tuned before this was fitting a loop that saw the world once per
quarter-metre. **Measure the camera's frame rate before tuning anything downstream of it** —
this cost days.

Also note the rate was NOT a fixed limit: the same camera hit 5.5 fps at times, tracking WiFi
latency. Averaging it into "a 1.4 fps camera" was wrong. Variable is worse than slow.

Switching the board to its own hotspot made it WORSE (1659 ms ping), so it was not congestion
on the house WiFi.

**Fix:** a Logitech C310 on USB, straight into the UNO Q -> 29.8 fps steady, no truncated
frames, no single-client limit, no WiFi. Full recipe, including why a passive USB-C OTG
adapter cannot work on this board and what does, in **`docs/usb-camera.md`**.

---

## [Camera] ESP32-CAM won't stay connected — brownout loop (POWERON_RESET)

**Symptom:** cam is "powered" but never shows up on the network / the operator Cam tab stays blank. Serial (or `/log`) shows it boot, reach `WiFi connecting`, then reset — repeatedly. Reset reason cycles `POWERON_RESET` (not a clean watchdog reset).

**Root cause:** **power sag.** The ESP32-CAM's biggest current spikes are camera-init and **WiFi TX** — so it browns out exactly when it tries to connect/stream, VDD drops below the reset threshold, and it power-cycles. Two culprits, in order of how often they bit us:
1. **Voltage drop in the connection to the cam**, not the supply itself. Measure the buck output vs the voltage **at the ESP `5V`/`GND` pins under load** — they should match within ~0.1V. We saw **buck 5.03V but ESP pins 4.57V** → a **0.46V drop across the connection**. The killer was **female dupont jumpers on the ESP pins** — tiny contact area, high resistance under load. **Fix: solder the 5V/GND wires directly to the ESP pins** (no jumpers/headers for power). That took the pins from 4.57V → **4.94V**, and a **dedicated buck feed via a soldered JST** got it to **5.24V** — solid.
2. **Sharing the buck rail** with the UNO Q + motors: their spikes sag the shared rail. **Give the cam its own buck output** (or a dedicated feed), plus the **470µF cap right at the cam's 5V/GND pins**.

**Also:** a low LiPo (≈40%) gives the buck less headroom → more sag. Keep the pack charged.

**Rule of thumb:** measure at the **ESP pins, under load** — want **~5.0V**. Idle 4.57V already means it'll brown out under the WiFi spike (AMS1117 dropout is ~4.4V). `reset_reason` in `/status` distinguishing `BROWNOUT`/`POWERON` from a clean run is the fastest tell.

## [Camera] Can't join WiFi even with good power — weak signal / WiFiMulti

**Symptom:** power is solid (stable 5.2V, `/log` shows one clean boot, no reset loop) but it sits at `WiFi connecting.....` forever (dots, never connects).

**Root cause:** the firmware only knew `FarmOS-AP` (down — UNO Q is in STA mode) and `home WiFi` (too weak where the cam sits). **WiFiMulti connects to the STRONGEST configured AP by RSSI — not a priority order.** Weak/absent known networks = stuck connecting.

**Fix:** add a **strong nearby AP** (we added `Bala`) to `wifiAddNetworks()` in `firmware/esp32_cam/esp32_cam.ino` (placeholder creds in git; real injected at flash time). Position that hotspot close to the robot so it wins the RSSI race. Both the cam **and the UNO Q** must be on the **same network** for the Cam tab to work (the UNO Q backend is the sole stream consumer).

## [Camera] Diagnose the cam over WiFi — no serial bridge needed

The cam firmware exposes diagnostics on **port 82** (added because debugging previously required the spare-Uno serial bridge every time):

```
http://farmcam.local:82/status   → {ssid,mode,rssi,ip,uptime_s,heap_free,heap_min,reset_reason,psram,camera}
http://farmcam.local:82/log      → recent boot log (WiFi result, IP, RSSI, mDNS)
```

- **`rssi`** = signal strength (weak? move the AP closer). **`reset_reason`** = `BROWNOUT`/`POWERON`/`PANIC`/`WDT` (why it last died). **`uptime_s`** tiny/not growing = it's resetting. **`heap_free`** low = crash risk.
- **SoftAP fallback:** if it can't join any WiFi within **25s**, it starts an open AP **`FarmCam-Diag`**. Connect a phone to it and open **`http://192.168.4.1:82/status`** — reachable even when it can't reach your WiFi. So you're never blind again.

## [Camera] Finding the cam's IP + flashing

- **IP:** it's DHCP (changes per boot) — don't chase a stale IP. Use **`farmcam.local`** (mDNS), or scan the subnet for its **Espressif MAC `e0:8c:fe:...`** (`arp -a | grep -i e0:8c:fe`, or from the UNO Q `ip neigh | grep -i e0:8c:fe`). Note a `REACHABLE`/ARP entry can be **stale** — confirm with an actual HTTP GET, not just ping.
- **Flashing** (no USB port on the AI-Thinker): spare Arduino Uno as USB-serial bridge — `RESET`→`GND` on the Uno; ESP `U0T`→Uno `D1`, `U0R`→Uno `D0`, common GND; `IO0`→`GND` for flash mode (**remove after** — floating = run mode). FQBN `esp32:esp32:esp32:UploadSpeed=115200,PSRAM=enabled,PartitionScheme=huge_app`. Firmware `config.xclk_freq_hz=20000000`, brownout detector disabled (`RTC_CNTL_BROWN_OUT_REG=0`). Manual reset timing is finicky — the power-on method (IO0 low, power off, start upload, power on when `Connecting....`) is the most reliable.

## [Drive] Robot won't drive straight / turns aren't 90° — see `drive-precision.md`

Whole-topic writeup: **`docs/farm-os/drive-precision.md`**. Summary so you don't re-derive it:

**Three stacked faults, none of them "the trim":**
1. **IBT-2 signal dupont jumpers degrade under vibration.** Same command drove 2.60 m dead
   straight early in a session and a **half circle** an hour later. Because it drifts, every
   parameter fitted during that window is garbage and never converges. Solder + strain-relieve.
2. **One wheel not touching.** A rigid 4-wheel frame rests on **3 of 4** points unless the
   axles are coplanar (~1 mm is enough — that's about all a tire deflects). That side loses
   ~half its traction → big veer, and **turns become forward arcs** because it can't
   counter-rotate. It also inflated the measured forward dead time 0.104 s → 0.524 s.
   Much less of a problem on soil, which conforms.
3. **Normal motor variance is 5–15%** at equal PWM (measured 5% here by counting free
   revolutions). That's what trim is for; anything much larger is a real fault.

**Plus one property you can't tune away:** skid-steer turns **coast** after the brake —
~75° at PWM 180, ~38° at PWM 120. Turn at a *lower* duty; a 90° turn at PWM 180 is mostly coast.

**Method that actually worked:**
- Run **`field_test.py diag`** first — `getDiag` proves what the MCU received and which pins
  it drove. Never tune control parameters before the command path is verified.
- **Free-running measurements hide load-dependent faults** (no current, no sag, no slip).
  Free-run said 5% deficit; loaded said 25%. Take both.
- Measure what's measurable: a **full circle** not "is that 90°"; **lateral offset in cm**
  not degrees; **are the two legs parallel** for a row change.
- Forward uses a **dead time** (positive offset), turns use a **coast** (negative offset).
  A negative result from `tsolve` is expected, not a bug.
- A 1.2% voltage difference cannot cause a 20% speed difference. Don't chase millivolts.

**Row change (`A→B→B1→C`) is the precision bottleneck:** both 90° turns go the *same* way,
so systematic overshoot **doubles** — 10°/turn leaves the next row 20° skew. Tune it as one
primitive: `field_test.py uturn`.

**Real fix:** **MPU6050 gyro (~₹150)** on the free `SDA`/`SCL`. Measure yaw and turn until the
angle is reached — surface-, battery- and coast-independent, and it removes the need for trim
altogether. Encoders fix *distance*, not heading (wheels slip by design in a turn).
