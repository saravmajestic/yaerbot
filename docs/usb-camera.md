# USB webcam on the Arduino UNO Q

How to get a USB webcam working on the UNO Q's Linux side, why the obvious approach fails,
and what it replaced.

**Status:** working. Logitech C310 delivering **~30 fps** through OpenCV inside the app
container (measured 2026-08-18: 179 frames in 6s at 640x480, slowest single read 37 ms).

**The app runs it at 320x240, natively — not downscaled.** The 640x480 figure above is from a
bare probe that never set `CAP_PROP_FRAME_WIDTH`. The property *does* work on this camera:
320x240 YUYV @30fps is a native C310 mode, so `camera.py`'s pinned size is what the sensor
actually streams and `_fit()` never resizes anything.

```bash
v4l2-ctl --device=/dev/video0 --get-fmt-video    # Width/Height : 320/240, 'YUYV'
```

**Do not raise the capture size to help the emitter model.** The deployed FOMO model's input
is **160x160** (`image_resize_mode: fit-shortest`), so every frame handed to the brick is
scaled to 160x160 before inference and 640x480 arrives identically to 320x240 — a strict
no-op. More model resolution has to come from a 160x160 *crop* of the lower frame (which
makes fit-shortest a no-op and lands the emitter 1:1, ~1.5x the model pixels) or from
retraining with a larger input in Edge Impulse. Check what the model actually wants before
changing capture settings:

```bash
ssh unoq 'curl -s http://127.0.0.1:1337/api/info'   # image_input_width/height, resize mode
```

Note also that `fit-shortest` centre-crops a 4:3 frame (320x240 -> 213x160 -> 160x160), so
the model never sees roughly the outer 8% of each side.

**`/dev/video` indices are not stable** — the C310 has been video2/video3 and video0/video1
on different boots of the same board. Always resolve by name; see `find_uvc_camera()`.

---

## Why we did this

The ESP32-CAM (MJPEG over WiFi) was the drip-vision camera for weeks. It was the root cause
of problems we spent three days attributing to the control code:

| | ESP32-CAM | Logitech C310 (USB) |
|---|---|---|
| frame rate | **0.7 – 6.7 fps**, erratic | **29.8 fps**, steady |
| ground covered per frame @0.17 m/s | 12 – 24 cm | **0.6 cm** |
| slowest single read | ~1456 ms | 37 ms |
| corrupt / green frames | ~5% of frames | none possible |
| ping / latency | 125 ms avg, 300 ms peak | n/a (USB) |
| stream clients | **one** — a second consumer wedges it | unlimited |

At 0.7 fps a robot creeping at 0.17 m/s travels 24 cm between measurements. No control gain,
lookahead weight or detector improvement can compensate for that; the tuning we did on
steering was fitting a loop that got one measurement per quarter-metre. **Measure the camera
frame rate before tuning any controller that depends on it.**

The ESP32-CAM's rate was also not a fixed hardware limit — it tracked WiFi latency, and the
same camera reached 5.5 fps at times. Averaging it and calling it "a 1.4 fps camera" was
wrong; it is *variable*, which is worse than slow because the control loop cannot plan for it.

---

## The trap: a passive USB-C OTG adapter CANNOT work

This is the part worth remembering, because everything about it looks like it should work.

`/sys/class/typec/port0/port_type` reads **`[sink]`**. The UNO Q's USB-C port is a
**sink-only** Type-C port: it presents Rd (the "I am the device" pull-down) on CC. A passive
OTG adapter *also* presents Rd. Rd-to-Rd means both ends are saying "you be the host", so:

```
/sys/class/typec/port0-partner        -> DOES NOT EXIST
lsusb                                 -> root hubs only
dmesg                                 -> no attach event at all
usb_vbus regulator                    -> 0 users, disabled
```

Nothing is detected, in either orientation, however long you wait. Confirmed over 45s of
replugging with a "pibox inspire" adapter.

**Do not chase this with a different passive adapter.** The role files are read-only even as
root, so it cannot be forced:

```bash
echo host   > /sys/class/typec/port0/data_role    # Permission denied
echo source > /sys/class/typec/port0/power_role   # Permission denied
```

## What DOES work: an adapter with external power in

```
external 5V ──> adapter's power-in port
                 adapter powers the webcam        (adapter supplies VBUS to the peripheral)
                 data lines ─────────────────────> UNO Q, dwc3 in host mode
                 board stays a POWER SINK          (which is all it is willing to be)
```

The board never has to source VBUS or win a CC negotiation. It stays a sink — exactly what
its port_type says — and only does data. Confirmed the moment external power was applied:

```
Bus 001 Device 002: ID 046d:081b Logitech, Inc. Webcam C310
usb 1-1: new high-speed USB device number 2 using xhci-hcd
usb 1-1: Found UVC 1.00 device <unnamed> (046d:081b)
/dev/video2  /dev/video3
```

So the shopping requirement is: **"USB-C hub / OTG adapter WITH a power-input port"**, not a
bare USB-C-to-A dongle. A community report on the Arduino forum matches — a plain adapter
failed OTG negotiation, one with a 100W charging port worked.

---

## Prerequisite: dwc3 must be in host mode

The QRB2210 has a driver bug: **booted without a USB-C cable attached, the controller comes
up in device mode instead of host.** Our board is powered from the 5V header, so it always
boots without a cable — exactly the affected case.

Check:

```bash
sudo cat /sys/kernel/debug/usb/4e00000.usb/mode      # want: host
```

If it says `device`, the fix is a boot-time debugfs write (systemd service, must start before
Docker so containers see the device):

```bash
echo host > /sys/kernel/debug/usb/4e00000.usb/mode
```

Arduino fixed this in OS images from Nov 2025. **Ours already has it** —
`/etc/buildinfo` reads `BUILD_ID=20251210-442`, and mode reads `host` with no intervention.
See github.com/Psalmustrack/arduino-uno-q-usb-fix.

Note `power_role=[sink]` does **not** mean host mode is impossible — that attribute is about
power delivery, not about whether xHCI can drive a peripheral. Reading it as a blocker cost
an hour.

---

## Driver and device notes

- `uvcvideo` is a module, not built in: `sudo modprobe uvcvideo`. It survives as long as the
  camera is attached; add it to `/etc/modules` if a reboot needs to bring it back.
- **THE /dev/video INDEX IS NOT STABLE — never hardcode it.** On one boot the C310 was
  `video2`/`video3` and the Qualcomm Venus codec was `video0`/`video1`. After unplugging the
  camera to mount it, they **swapped**: the camera came back as `video0`/`video1` and
  `video2` became the H.264 encoder. Opening the wrong index either fails outright or, worse,
  succeeds on the codec and never delivers a frame.

  Identify by CAPABILITY instead — `find_uvc_camera()` in `console/python/vision/camera.py`
  parses `v4l2-ctl --list-devices`, which groups nodes under their driver: the codecs
  announce `platform:qcom-venus`, a UVC camera announces its USB address. It falls back to
  probing each node for a real frame. **In the console's Cam field, enter `usb`** — not a
  number.

  ```
  UVC Camera (046d:081b) (usb-xhci-hcd.2.auto-1):   <- the camera
  Qualcomm Venus video encoder (platform:qcom-venus): <- NOT a camera
  ```
- **The app container already has access.** It is not privileged and has no explicit device
  mappings, but its cgroup rules include `c 81:* rmw` — all major-81 (video) devices — and
  `/dev` is shared, so a camera plugged in *after* the container started is visible inside it
  with no restart.

## C310 formats (v4l2-ctl -d /dev/video2 --list-formats-ext)

`YUYV` only (no MJPG exposed). Sizes include **320x240** — which is what the ESP32-CAM
delivered and what every detector threshold was calibrated against — plus 640x480 at 30 fps.

**`cap.set(CAP_PROP_FRAME_WIDTH, 320)` is IGNORED by this camera** — it reports 640x480
regardless. That matters more than it looks: `_TUBE_MIN_W_PX`, band drift limits and
`_profile_line`'s min/max widths are pixel constants measured at 320x240, so a tube 38px wide
at QVGA reads as 76px at VGA and gets thrown out as implausible.

FIXED: `FrameSource` pins `FRAME_W, FRAME_H = 320, 240` and **enforces it on every frame**
(resizing when the driver disagrees) rather than requesting and hoping. `bus.resized` counts
how often it had to. Confirmed on the board: frames arrive as `(240, 320, 3)` at 30.0 fps.

At 640x480 `detect_tube` costs **6.0 ms/frame**, so at 30 fps the detector uses ~18% of one
core. Ample headroom.

---

## Power

The camera declares **Bus Powered, MaxPower 500 mA** (real draw is usually 150–250 mA;
measure it in series and size the fuse from that).

Feeding the adapter from the robot's buck:

- **Fuse the branch on its own** — ~1 A. Per the rule that came out of the 12V feed fire:
  every branch gets its own fuse sized to its own wire, never hung off another branch.
- **Raw 5 V into a USB-C power port may be refused.** USB-C sinks generally want the source
  to signal on CC; a mains charger does, a buck's bare 5 V does not. Reliable route: feed a
  **USB-A socket** from the buck and use a **USB-A → USB-C cable**, which carries the required
  56 kΩ pull-up per spec. Alternatively use a 5 V USB-C charger module that signals properly.
- **Ground gets better, not worse.** Everything then shares the robot's single ground. The
  ground-loop warning (see `AGENTS.md`) is about a *laptop's* USB being a separate ground
  domain;
  powering from the robot's own buck removes that.
- **Expect the board not to power off cleanly.** VBUS present at the USB-C port is what
  "keeps it from staying off". Shutdown stays: `sudo halt`, wait for the LED, cut the LiPo.
- If the C310 becomes the robot's camera, the **ESP32-CAM's buck feed can be removed**, which
  roughly pays for the webcam's draw and deletes a whole failure domain.

---

## Reproducing from scratch

```bash
# 1. host mode (should already be 'host' on OS images >= Nov 2025)
sudo cat /sys/kernel/debug/usb/4e00000.usb/mode

# 2. driver
sudo modprobe uvcvideo

# 3. plug in: webcam -> adapter -> UNO Q USB-C, WITH external power into the adapter
sudo lsusb                      # expect the camera's VID:PID
ls /dev/video*                  # expect video2 (video0/1 are the Venus codec)
ls -d /sys/class/typec/port0-partner   # sanity: still 'none' — the board is a sink, that is fine

# 4. prove frames, from inside the app container
docker exec motor-control-main-1 python3 - <<'EOF'
import cv2, time
cap = cv2.VideoCapture(2)
for _ in range(5): cap.read()
t0=time.time(); n=0
while time.time()-t0 < 6:
    ok,f = cap.read(); n += ok
print("%.1f fps" % (n/6.0))
cap.release()
EOF
```

## Diagnosing a camera that does not appear

| symptom | meaning |
|---|---|
| `lsusb` shows only root hubs, no dmesg attach event | passive adapter — no CC signalling, or no external power into the adapter |
| `port0-partner` missing | normal for this board; it is a sink-only port and never sees a "partner" |
| mode reads `device` | the QRB2210 boot bug — write `host` to the debugfs mode file |
| device enumerates but `/dev/videoN` missing | `uvcvideo` not loaded |
| container cannot open it | check the cgroup rule includes `c 81:* rmw` |

## Geometry must be re-measured after mounting

`_ARRIVE_EXTRA_M = 0.38` and the steering lookahead weights come from the ESP32-CAM's
position (15 cm high, 34 cm strip, bottom edge 20 cm ahead of the wheel contact, +5 cm to the
front axle, +13 cm to the pivot axis = half of the 26 cm wheelbase). A different camera at a
different height and angle invalidates all of it. Re-measure: what ground distance sits at the
bottom edge of the live view, and how far that is from the pivot axis.
