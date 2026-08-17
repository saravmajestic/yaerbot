import base64
import json
import os
import socket
import sqlite3
import struct
import subprocess
import threading
import time
import urllib.request
from datetime import datetime, timezone

from arduino.app_utils import Bridge, App
from arduino.app_bricks.web_ui import WebUI

# Vision is OPTIONAL — guarded so the operator app runs even if OpenCV / the
# vision package aren't on the board yet. The raw camera feed (an <img> straight
# to the ESP32-CAM) works regardless; only the detection overlay + drip-follow
# need this. To enable on-device: `pip install opencv-python-headless` in the
# container and bundle the repo's vision/ folder next to this app.
_VISION_OK = False
try:
    import sys as _sys
    _here = os.path.dirname(os.path.abspath(__file__))
    # `_here` (contains the vision/ package) + repo root (local dev). NOTE: do NOT
    # add _here/vision to the path — that shadows the `vision` package with its own
    # vision.py module and breaks `from vision.vision import ...`.
    for _p in (_here,
               os.path.abspath(os.path.join(_here, "..", "..", ".."))):  # repo root (local dev)
        if _p not in _sys.path:
            _sys.path.insert(0, _p)
    import cv2  # noqa: F401
    from vision.vision import detect_tube, detect_emitter, detect_crossing, draw_overlay
    from vision.camera import FrameSource
    # Optional on-device ML emitter model (Edge Impulse via the object_detection
    # brick). Self-guards; falls back to classical detect_emitter if no model.
    from vision.emitter_ml import detect_emitter_ml, ml_available
    _VISION_OK = True
except Exception as _e:                      # noqa: BLE001
    print("vision unavailable (raw feed only): %s" % _e, flush=True)

# Act 2/4 planner + executor + report. Guarded the same way as vision so a missing
# farmos/ can't stop the console from starting.
_PLOT_OK = False
try:
    from farmos import SeedPlan, plan_boustrophedon, execute
    from farmos.executor import RunLog
    from farmos.path import plan_summary
    from farmos.report import render_svg
    from farmos.robot_io import BridgeRobot
    _PLOT_OK = True
except Exception as _e:                      # noqa: BLE001
    print("farmos planner unavailable (Seed tab plot mode off): %s" % _e, flush=True)

ui = WebUI()


def log(msg):
    print("[%s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


def logged(name, handler):
    """Wrap a UI message handler so every incoming request + payload is logged."""
    def wrapper(client, data):
        log("rx %-13s %s" % (name, data))
        return handler(client, data)
    return wrapper

# ── Host control (power + network) ─────────────────────────────────────────
# This app runs in an unprivileged container, so it can't power off the host
# or change WiFi directly. It asks the root-side helper (server/power_helper.py)
# over the container's default gateway. Token must match TOKEN in the helper.
HELPER_TOKEN = "farmos-power"
HELPER_PORT  = 7999

def _host_gateway():
    """The container's default gateway IS the UNO Q host.

    Read /proc/net/route rather than shelling out: this image has no `ip`
    binary, so the old `ip route` call always failed and silently fell through
    to a hardcoded address that only happened to be right.
    """
    try:
        with open("/proc/net/route") as f:
            for line in f.read().splitlines()[1:]:
                fields = line.split()
                if fields[1] == "00000000":          # destination 0.0.0.0 = default
                    return socket.inet_ntoa(struct.pack("<L", int(fields[2], 16)))
    except Exception:                                 # noqa: BLE001
        pass
    return "172.19.0.1"

def _helper_request(action):
    gw = _host_gateway()
    url = "http://%s:%d/%s" % (gw, HELPER_PORT, action)
    print("helper: %s -> POST %s" % (action, url), flush=True)
    body = json.dumps({"token": HELPER_TOKEN}).encode()
    req = urllib.request.Request(
        url, data=body, headers={"Content-Type": "application/json"}, method="POST"
    )
    try:
        resp = urllib.request.urlopen(req, timeout=3)
        print("helper: %s -> replied %s" % (action, resp.status), flush=True)
    except Exception as e:
        print("helper: %s request FAILED: %s" % (action, e), flush=True)

_speed = 180  # default ~70% (0-255)

# Trim: reduce the faster side until robot goes straight.
# Robot drifts RIGHT → right motors are faster → reduce RIGHT_TRIM.
# Adjust in 0.05 steps and re-deploy until straight.
LEFT_TRIM  = 0.83      # 2026-08-16: was 0.77; matched to the field-test `ltrim=0.83`
RIGHT_TRIM = 1.00      # that the row/cycle runs are calibrated with. The two had
                       # drifted apart, so the Drive tab steered differently from
                       # field_test.py on the same ground.

def trimmed(left, right):
    return int(left * LEFT_TRIM), int(right * RIGHT_TRIM)


# Calibration measured 2026-08-11 (hard floor, 3S ~12V). These are surface- and
# hardware-specific — recalibrate after ANY mechanical/wiring change with
# scripts/field_test.py (solve/tsolve). See docs/farm-os/drive-precision.md.
# batt_comp stays off: the A4 divider still reads low and wanders.
CAL = {
    "pwm": 180, "speed": 0.628, "startup": 0.099,
    "ltrim": 0.83, "rtrim": 1.00,
    "turn_pwm": 120, "tdps": 45.2, "tstartup": -0.80, "tramp": 0.0,
    "creep": None,      # m/s at _BASE_PWM (tube-following). None -> see _DRIP_SPEED_MPS
}
# Synced 2026-08-16 to the numbers actually validated on soil with field_test.py —
# this block had drifted and the Seed tab was running stale calibration:
#   ltrim   0.75 -> 0.83   (also matches LEFT_TRIM above; there were THREE values)
#   speed   0.616 -> 0.628
#   startup 0.104 -> 0.099
#   tstartup -0.75 -> -0.80
#   tdps    51.0 -> 45.2   <- see below, this one is not a fresh measurement
#
# On tdps: the operator found `turn=125` produced a correct row change on the ground,
# validated by the two rows actually coming out parallel — better evidence than any of
# the single-angle fits before it. turn=125 @ tdps=62.8 commands -0.80 + 125/62.8 =
# 1.190s, so tdps = 90/(1.190+0.80) = 45.2 makes `turn=90` command that SAME 1.190s.
# Identical motor behaviour, but the geometry stays honest, so uturn/plan/Seed-tab all
# inherit the correction instead of each needing its own fudge.
#
# This is still an OPEN-LOOP timed pivot and it will keep drifting with the surface —
# a skid-steer point turn rotates by scrubbing the wheels sideways, so hard floor and
# loose tilth genuinely differ. The fix is closed-loop heading from a gyro, not a
# better constant. See docs/farm-os/drive-precision.md.


# ── Diagnostics: what THIS side last sent ──────────────────────────────────
# The Diag tab traces one command end to end (browser → console → MCU → driver
# pins → motor current). Stages 3-5 come from the MCU's getDiag; this records
# stage 2, so a mismatch pins the break to a specific hop.
_last_cmd = {"src": None, "direction": None, "left": None, "right": None,
             "speed": None, "at": None}


def _drive(left, right, src, direction=None):
    """Every motor command goes through here so Diag always has the real value."""
    _last_cmd.update(src=src, direction=direction, left=left, right=right,
                     speed=_speed, at=time.time())
    Bridge.call("setMotors", left, right)


def _drive_stop(src):
    _last_cmd.update(src=src, direction="stop", left=0, right=0,
                     speed=_speed, at=time.time())
    Bridge.call("stop")

DIRECTIONS = {
    "forward":  lambda s: trimmed( s,  s),
    "backward": lambda s: trimmed(-s, -s),
    "left":     lambda s: trimmed(-s,  s),      # pivot, in place
    "right":    lambda s: trimmed( s, -s),      # pivot, in place
}


def on_motor_cmd(client, data):
    direction = data.get("direction", "")
    fn = DIRECTIONS.get(direction)
    if fn:
        left, right = fn(_speed)
        _drive(left, right, "drive", direction)


def on_motor_stop(client, data):
    _drive_stop("drive")


def on_set_speed(client, data):
    global _speed
    _speed = max(60, min(255, int(data.get("speed", 180))))


def on_shutdown(client, data):
    Bridge.call("stop")          # stop motors before powering down
    _helper_request("shutdown")

def on_reboot(client, data):
    Bridge.call("stop")
    _helper_request("reboot")

def on_connect_wifi(client, data):
    _helper_request("wifi")      # switch to home WiFi (reverts to hotspot on fail)

def on_start_hotspot(client, data):
    _helper_request("hotspot")   # force the FarmOS-AP hotspot


# ── Seeder ──────────────────────────────────────────────────────────────────
# The timed seconds-based single-row mode was removed: it predated the calibrated
# drive model and expressed spacing as a duration, so it could not hold real
# spacing. Plot seeding (see the plot_* handlers below) replaces it — it plans in
# metres and drives a serpentine with the calibrated dead-time/coast models.
def on_plant_once(client, data):
    """Manual single plant (test button) — refused while a plot run is active."""
    if _run["state"] in ("running", "paused"):
        return
    Bridge.call("plantSeed")


# ── Soil sensing (Stage 2) ──────────────────────────────────────────────────
# Reads the three DIY probes over RPC and logs samples to SQLite (schema shared
# with tests/test_soil_log.py). NOTE: getTemperature blocks ~750ms (DallasTemp),
# so the UI polls soil ONLY while the Soil tab is open — never in the 2s stats.
# GPS lat/lng are NULL until a getGPS RPC lands (BN-880 not yet exposed).
SOIL_DB = os.path.expanduser("~/farm_os.db")
SOIL_SCHEMA = """
CREATE TABLE IF NOT EXISTS soil_readings (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp TEXT NOT NULL, lat REAL, lng REAL,
    moisture_a0 INTEGER, moisture_a1 INTEGER, temp_c REAL, ec_raw INTEGER
);
"""
_soil = {"surveying": False, "interval_s": 5}
_soil_thread = None

def _decode(v):
    return v.decode() if isinstance(v, bytes) else v

def _soil_conn():
    conn = sqlite3.connect(SOIL_DB)
    conn.execute(SOIL_SCHEMA)
    conn.commit()
    return conn

def _moist_pct(raw):
    # rough map: dry ≈ 12000, wet ≈ 4000 (14-bit ADC; lower = wetter)
    if raw is None:
        return None
    return max(0, min(100, round((12000 - raw) / (12000 - 4000) * 100)))

def read_soil():
    """Unified read of all three probes; each field is None on read failure."""
    try:
        moist = json.loads(_decode(Bridge.call("getMoisture")))
    except Exception:
        moist = {}
    try:
        temp = float(_decode(Bridge.call("getTemperature")))
        if temp <= -100:            # -127 = no DS18B20 found
            temp = None
    except Exception:
        temp = None
    try:
        ec = int(_decode(Bridge.call("getEC")))
    except Exception:
        ec = None
    a0, a1 = moist.get("a0"), moist.get("a1")
    return {
        "moisture_a0": a0, "moisture_a1": a1,
        "moist_pct_0": _moist_pct(a0), "moist_pct_1": _moist_pct(a1),
        "temp_c": temp, "ec_raw": ec,
    }

def _log_soil(r, lat=None, lng=None):
    conn = _soil_conn()
    conn.execute(
        "INSERT INTO soil_readings (timestamp,lat,lng,moisture_a0,moisture_a1,temp_c,ec_raw) "
        "VALUES (?,?,?,?,?,?,?)",
        (datetime.now(timezone.utc).isoformat(), lat, lng,
         r["moisture_a0"], r["moisture_a1"], r["temp_c"], r["ec_raw"]),
    )
    conn.commit()
    n = conn.execute("SELECT COUNT(*) FROM soil_readings").fetchone()[0]
    conn.close()
    return n

def _soil_count():
    try:
        conn = _soil_conn()
        n = conn.execute("SELECT COUNT(*) FROM soil_readings").fetchone()[0]
        conn.close()
        return n
    except Exception:
        return 0

def on_get_soil(client, data):
    r = read_soil()
    r["surveying"] = _soil["surveying"]
    r["logged"] = _soil_count()
    ui.send_message("soil", r)

def on_soil_sample(client, data):
    r = read_soil()
    n = _log_soil(r)
    r["logged"] = n
    r["sampled"] = True
    ui.send_message("soil", r)

def _survey_loop():
    try:
        while _soil["surveying"]:
            _log_soil(read_soil())
            # sleep in chunks so Stop is responsive
            end = time.time() + _soil["interval_s"]
            while time.time() < end and _soil["surveying"]:
                time.sleep(0.1)
    finally:
        _soil["surveying"] = False

def on_survey_start(client, data):
    global _soil_thread
    _soil["interval_s"] = max(2, int(data.get("interval_s", _soil["interval_s"])))
    if _soil["surveying"]:
        return
    _soil["surveying"] = True
    _soil_thread = threading.Thread(target=_survey_loop, daemon=True)
    _soil_thread.start()

def on_survey_stop(client, data):
    _soil["surveying"] = False


# ── Camera + vision (Stage 5) ────────────────────────────────────────────────
# The UNO Q is the SINGLE consumer of the ESP32-CAM stream (the cam is weak and
# chokes serving two clients — that was the browser lag). One background loop
# reads frames, runs detect_tube/detect_emitter at full rate, steers+plants in
# drip mode, AND — only while the Camera tab is open — pushes annotated JPEG
# frames to the browser over the socket at a throttled rate. The browser never
# touches the cam directly.
_cam = {"url": "", "w": 0, "h": 0, "tube": None, "emitter": None, "watch": 0.0,
        "last_frame": 0.0}          # monotonic-ish stamp of the last SUCCESSFUL read
# How long without a decoded frame before the stream counts as dead. The ESP32-CAM's
# MJPEG stream can stop delivering while the socket stays open — read() then returns
# False forever and the loop livelocks: thread alive, nothing arriving. See _cam_loop.
_CAM_STALE_S = 5.0
_CAM_REOPEN_TRIES = 3
# Drip seeding (Act 3). The trigger is the CAMERA, not odometry: plant at each
# emitter the model finds. `emitter_gap` is the operator's APPROXIMATE spacing —
# it isn't used to place seeds, only to (a) reject a second detection of the same
# emitter and (b) estimate counts/time for the UI.
# The seeder arm carries 2 outlets 180° apart and one solenoid fires both, so a
# single plantSeed drops 2 seeds. `angle` rotates that pair: 0° lays them along
# the drip line, 90° across it. The opposite outlet always lands at angle+180.
# `angles` are ARM POSITIONS to plant at, in order. Each one drops 2 seeds (the
# two outlets are 180 deg apart and one solenoid fires both), so [0, 90] plants a
# 4-seed cross per emitter: 0/180, then rotate, then 90/270.
_drip = {"emitter_gap": 0.40, "angles": [0, 90],
         # Multi-lateral (Act 3 extension). laterals=1 keeps the original behaviour:
         # one row, then done. >1 makes the robot SEARCH for the next lateral rather
         # than compute where it should be — turn off the row, drive across until the
         # camera sees a crossing tube, turn onto it. That needs no row spacing and no
         # assumption that the laterals are parallel, which real drip layouts rarely are.
         "laterals": 1}
# Between-lateral search tuning.
_TRAVERSE_MIN_M   = 0.10   # ignore crossings until this far off the old row.
                           # WAS 0.60, as a guard against re-latching onto the lateral we
                           # just left. It cost us the row change on 2026-08-17: the real
                           # next lateral was seen at 0.34m and 0.53m, steady and strong,
                           # and BOTH were discarded as "still inside min-traverse" — the
                           # robot then turned onto something at 0.82m. Two reasons this
                           # is now 0.10:
                           #   * the distance it gates on is time x an unmeasured creep
                           #     speed, so it is not trustworthy at this scale anyway;
                           #   * the guard has never actually been needed. After the
                           #     90 deg turn the old row is behind the robot, and every
                           #     run logs found=False for the first samples (0.00m, 0.17m)
                           #     — the camera looks ahead and simply does not see it.
                           # 0.10 keeps a floor against detecting the old row in the very
                           # first frames while the robot is still sitting on it.
_TRAVERSE_NEAR    = 0.60   # `nearness` at which the tube is close enough to turn onto
# TURN ON APPROACH, NOT ON A SINGLE GOOD FRAME. The 03:33 run detected crossings at
# nearness 0.53, 0.39, 0.91, then 0.08, 0.06, 0.19 and never turned: the old rule wanted
# three CONSECUTIVE frames agreeing within 40px, and one dropped frame reset the count to
# zero every time. But a lateral the robot is driving towards has a signature no soil
# artefact has — it moves steadily DOWN the frame. So instead of demanding consecutive
# agreement, keep a short history and require the trend: several sightings within a couple
# of seconds whose y has grown. Dropouts no longer reset anything; they just thin the
# history.
_TRAVERSE_TRACK_S     = 2.5   # how much sighting history counts as "recent"
_TRAVERSE_MIN_SIGHTS  = 4     # sightings needed inside that window
_TRAVERSE_APPROACH_PX = 25    # y must have grown by this much across the window
_ARM_DWELL_S  = 0.6
# plantSeed on the MCU is drop(500) + settle(100) + punch(500) + fill(400) = 1.5s.
# A dry run sleeps the same so its timing matches a real run.
_PLANT_SIM_S  = 2.0      # let the spool servo reach the angle before dropping
_MIN_REPLANT_M = 0.08    # absolute floor; see _min_replant() for the run-time value


def _min_replant():
    """Distance that must pass before the NEXT emitter can be planted.

    Still not a trigger — the model decides where emitters are, and a missed one just
    means the next is found normally. This only stops ONE emitter being counted twice,
    which the 03:33 run did: stops at 2.73m and 2.84m, 11cm apart on a line whose
    emitters sit 40cm apart, because the emitter left the frame during a tube dropout
    and re-armed on the way back in. Half the operator's spacing is comfortably below
    any real gap and comfortably above that.
    """
    gap = float(_drip.get("emitter_gap") or 0.0)
    return max(_MIN_REPLANT_M, 0.5 * gap)


def _traverse_max():
    """How far to search for the next lateral before giving up.

    Was a flat 4.0m, which at the current speed estimate is nearly 6m of real ground —
    the robot drives that far with nothing in view, which reads as a runaway. The
    operator has already told us how far apart the rows are, so allow three of them.
    """
    return max(1.5, 3.0 * float(_plot.get("row_gap") or 0.7))
_cam_thread = None
_BROWSER_FPS = 6                    # annotated frames/s pushed to the browser
_WATCH_TTL   = 2.0                 # keep pushing this long after the last get_vision poll

# Dataset capture: save RAW frames at an interval while driving the drip line,
# to build the emitter-model training set. Start/stop + verify from the UI.
# Saved under the app dir (mounted to the host ~/motor-control/captures) so they
# PERSIST across container restarts and are pullable from the host.
_CAP_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "captures"))
_capture = {"on": False, "interval": 2.0, "count": 0, "last": 0.0}

# drip-follow tunables (mirror vision/tube_follow.py)
_BASE_PWM, _STEER_GAIN, _EMIT_CONF, _EMIT_COOLDOWN = 77, 6, 0.55, 3.0
# WHERE in frame the emitter must be before we stop for it. The camera looks down AND
# ahead, so an emitter first appears near the TOP of the frame — a metre or more away.
# Confidence alone as the trigger meant the robot stopped the instant each emitter
# entered view, at a different distance every time, and would have dropped seed nowhere
# near the emitter. Lower in frame = nearer, so hold off until the detection's centre is
# below this fraction of the frame height.
_EMIT_MIN_Y_FRAC = 0.55


def _emitter_in_reach(emit, h):
    """True when the detected emitter is near enough to be under the seeder."""
    pos = emit.get("position")
    if not pos or not h:
        return False
    return pos[1] >= h * _EMIT_MIN_Y_FRAC
# Steering, retuned 2026-08-17 after a field run oscillated badly — corrections swung
# +2.9 / -1.0 / +0.2 / -2.0 / -3.2 within a couple of seconds, losing the tube about
# twice a second. Two faults, opposite in sign:
#
#   TOO WEAK at small error — a correction of 0.3 asks for 3 PWM of differential on a
#   base of 77. That does not overcome stiction on soil, so the robot drifts, doing
#   nothing, until the error is large.
#   TOO STRONG at large error — 10 x correction at ~10 fps (the ML detector costs a
#   ~70-90ms round trip) overshoots past centre, and P-only control with that much lag
#   turns drift into a limit cycle.
#
# So: lower the proportional gain, add a floor that actually moves the robot, and use
# the tube's TILT as a damping term. detect_tube has always returned `angle_deg` and
# nothing ever read it — steering on position alone is why it hunts. Tilt says whether
# we are converging or diverging, which is the derivative term this loop lacked.
_STEER_MIN_DIFF = 11    # PWM differential floor once past the deadband (beats stiction)
_STEER_DEADBAND = 0.25  # ignore correction noise below this — do not chase pixels
_STEER_ANG_GAIN = 0.0   # DISABLED 2026-08-17: tilt damping made steering WORSE in
                        # the field. The sign convention of detect_tube's angle_deg
                        # was never verified against the robot, and a wrong-signed
                        # derivative term amplifies the hunting it was meant to damp.
                        # Do not re-enable without checking the sign on hardware.


# ── Temporal gate on the tube reading ────────────────────────────────────────
# A per-frame detector, however good, cannot rule out a reading that is physically
# impossible. The 2026-08-17 run logged the tube at x = 233, 290, 158, 61, 256, 168,
# 14 within about two seconds — the robot creeps at ~0.2 m/s and cannot cross the frame
# in 100ms, so most of those were false whatever the detector said. This keeps the last
# accepted column and rejects anything that jumps further than the robot could have
# moved; after enough consecutive rejections it gives up and re-acquires, so a genuine
# re-acquisition after losing the tube still works.
_TRACK_MAX_JUMP_PX = 70     # per frame at ~10fps; a real tube drifts, it does not teleport
_TRACK_RELOCK_N    = 6      # consecutive rejects before believing the new position
_TRACK_STALE_S     = 1.5    # no accepted reading for this long -> accept the next one
# PLAUSIBLE FOR THIS CAMERA. The detector reports what the profile says; how wide the
# tube can look from this camera at this height is the robot's knowledge, not the
# detector's. Measured across every real following frame: 38, 40, 46, 46, 62, 68, 70,
# 70, 78 px. The false positive that sent the alignment loop chasing bare soil on
# 2026-08-17 was 20px. The gap is wide enough to gate on and it costs nothing.
_TUBE_MIN_W_PX   = 30
_TUBE_MIN_SIGMA  = 2.5
_track = {"x": None, "rejects": 0, "last_ok": 0.0}


def _tube_plausible(tube):
    """Is this reading the right SHAPE to be our tube, seen from our camera?"""
    return (tube["found"] and (tube["width"] or 0) >= _TUBE_MIN_W_PX
            and tube["strength"] >= _TUBE_MIN_SIGMA)


def _track_reset():
    _track.update(x=None, rejects=0, last_ok=0.0)


def _track_tube(tube, w, now):
    """Accept, reject or re-acquire this frame's tube reading."""
    if not _tube_plausible(tube):
        _track["rejects"] = 0            # nothing to disagree with; do not count it
        out = dict(tube)
        if tube["found"]:                # found, but not tube-shaped: say so
            out.update(found=False, correction=0.0, implausible=True)
        return out
    x = tube["tube_x"]
    prev = _track["x"]
    stale = (now - _track["last_ok"]) > _TRACK_STALE_S
    if prev is None or stale or abs(x - prev) <= _TRACK_MAX_JUMP_PX \
            or _track["rejects"] >= _TRACK_RELOCK_N:
        _track.update(x=x, rejects=0, last_ok=now)
        return tube
    _track["rejects"] += 1
    out = dict(tube)
    out.update(found=False, correction=0.0, rejected_x=x,
               reject_run=_track["rejects"])
    return out


def _steer(tube):
    """Differential PWM for the current tube reading. Position + tilt damping."""
    c = tube.get("correction") or 0.0
    if abs(c) < _STEER_DEADBAND:
        c = 0.0
    d = _STEER_GAIN * c

    # Tilt: detect_tube now returns SIGNED tilt from vertical (0 = aligned), so no
    # conversion. It says which way the robot is heading off BEFORE the offset has
    # grown — the early warning position feedback alone cannot give.
    a = tube.get("angle_deg")
    if a is not None:
        d += _STEER_ANG_GAIN * max(-45.0, min(45.0, a))

    if d and abs(d) < _STEER_MIN_DIFF:                 # floor, so small errors DO move it
        d = _STEER_MIN_DIFF if d > 0 else -_STEER_MIN_DIFF
    return int(max(-60, min(60, d)))                   # cap: never reverse a wheel
# Tube-following creeps at _BASE_PWM, well below the PWM the drive was calibrated at.
# Scaling the calibrated cruise speed by the duty ratio (0.628 * 77/180 = 0.269 m/s)
# OVER-reports badly: a run told to cover 5 m declared itself finished after about 3 m
# of real ground. Speed is not linear in PWM — near the deadband most of the duty goes
# into overcoming stiction, not into motion.
#
# Two runs have now calibrated it by observation, and they disagreed because the robot
# was stuttering — every tube dropout costs acceleration time, so the effective speed
# depended on how badly the detector was misbehaving that run:
#     assumed 0.269 -> commanded 5m, covered ~3m  => ~0.161 m/s
#     assumed 0.161 -> commanded 5m, covered ~7m  => ~0.225 m/s   (steady following)
# The second is from the run where the tube was actually being tracked, so it is the
# better number. It is still an observation, not a measurement. MEASURE IT:
#     ssh unoq 'python3 ~/motor-control/scripts/field_test.py fwd 77 10 0.83'
#     creep = (metres the robot actually moved) / 10   -> put it in CAL["creep"]
_DRIP_SPEED_MPS = CAL.get("creep") or 0.225

def _moisture_min():
    try:
        m = json.loads(_decode(Bridge.call("getMoisture")))
        return min(m.get("a0", 16383), m.get("a1", 16383))
    except Exception:
        return None

_mdns_cache = {}          # name -> (expires_at, ip)
_MDNS_TTL = 60.0          # short: the cam's IP changes with the network it joins


def _resolve_mdns(name):
    """Resolve a *.local name to an IP, asking the host helper to do it.

    This container's DNS is Docker's internal resolver, which has no mDNS, and
    multicast doesn't cross the bridge to wlan0 — so `farmcam.local` is
    unresolvable in here. The host runs avahi and can resolve it, so we ask it.
    Returns the name unchanged if that fails, leaving the error to the caller.
    """
    now = time.time()
    hit = _mdns_cache.get(name)
    if hit and hit[0] > now:
        return hit[1]
    url = "http://%s:%d/resolve?name=%s" % (_host_gateway(), HELPER_PORT, name)
    try:
        with urllib.request.urlopen(url, timeout=3) as r:
            ip = json.loads(r.read()).get("ip")
        if ip:
            _mdns_cache[name] = (now + _MDNS_TTL, ip)
            log("resolved %s -> %s (via host)" % (name, ip))
            return ip
        log("resolve %s: helper returned no ip" % name)
    except Exception as e:                    # noqa: BLE001
        log("resolve %s failed: %s" % (name, e))
    return name


def _host(hostport):
    """Swap a bare *.local host for its IP, preserving any :port suffix."""
    host, _, port = hostport.partition(":")
    if host.endswith(".local"):
        host = _resolve_mdns(host)
    return host + (":" + port if port else "")


def _stream_url(base):
    """Accept 'ip', 'host.local', 'http://…', or a full URL -> MJPEG stream URL."""
    b = base.strip()
    if not b:
        return ""
    if b.startswith("http"):
        rest = b.split("://", 1)[1]
        hostport, slash, tail = rest.partition("/")
        b = "http://" + _host(hostport) + slash + tail
        return b if "/stream" in b else b.rstrip("/") + ":81/stream"
    hostport = _host(b)                       # bare 'ip' / 'host.local' / either with :port
    if ":" not in hostport:
        hostport += ":81"                     # the ESP32-CAM's stream port
    return "http://%s/stream" % hostport

def _cam_loop(url):
    travelled = 0.0             # estimated distance along the lateral this run
    last_plant_at = -1e9        # travelled-at-last-plant, so the first emitter is never gated
    armed = True                # detection-edge debounce (see the drip branch)
    driving_since = None        # when the current drive segment began
    tube_seen = None            # last tube["found"], for EDGE logging (None = no edge yet)
    # Multi-lateral drip state. phase: "follow" (down a row) | "traverse" (crossing to
    # the next one). `lateral` is 0-based and also picks the turn direction, so the
    # path serpentines instead of circling.
    phase = "follow"
    lateral = 0
    traversed = 0.0
    cross_hist = []             # recent crossing sightings: (time, tube_y)
    last_cross_log = 0.0
    prev_run_state = None       # to detect a FRESH run and reset per-run counters
    last_push = 0.0
    push_interval = 1.0 / _BROWSER_FPS
    try:
        src = FrameSource(_stream_url(url))
    except Exception as e:
        print("camera open failed: %s" % e, flush=True)
        return
    print("camera loop (sole consumer): %s  [emitter: %s]"
          % (_stream_url(url), "ML/Edge-Impulse" if ml_available() else "classical CV"),
          flush=True)
    fail_since = None
    reopens = 0
    while _cam["url"] == url and _VISION_OK:
        ok, frame = src.read()
        if not ok:
            # NO FRAME = NO EYES. Stop the motors before anything else.
            # This branch used to `continue` straight past the drive logic, which
            # left the LAST setMotors command running: on 2026-08-16 a stream that
            # went stale mid-run drove the robot blind for 22s, past its stop
            # distance, because `travelled` also only accumulates below. Same
            # fail-safe as "tube lost -> stop", one level lower down.
            if _run["state"] == "running" and _run["mode"] in ("scan", "drip"):
                if driving_since is not None:
                    log("camera stopped delivering frames mid-run — STOPPING (blind)")
                _drive_stop("cam-lost")
                driving_since = None
            # A dead MJPEG stream returns False forever while the socket stays open,
            # so this thread would stay ALIVE but useless — and on_set_camera refused
            # to start a replacement precisely because a thread was alive. Result:
            # Connect did nothing, permanently (2026-08-15, seven clicks, no effect).
            # So: reopen the stream, and if that keeps failing, DIE so Connect works.
            fail_since = fail_since or time.time()
            if time.time() - fail_since > _CAM_STALE_S:
                reopens += 1
                if reopens > _CAM_REOPEN_TRIES:
                    log("camera stream dead and %d reopens failed — dropping the loop "
                        "so Connect can start a fresh one" % _CAM_REOPEN_TRIES)
                    _cam["url"] = ""          # UI shows disconnected; next Connect is clean
                    break
                log("camera stream stale for %.0fs — reopening (attempt %d/%d)"
                    % (_CAM_STALE_S, reopens, _CAM_REOPEN_TRIES))
                try:
                    src.release()
                except Exception:             # noqa: BLE001
                    pass
                try:
                    src = FrameSource(_stream_url(url))
                    log("camera stream reopened")
                except Exception as e:        # noqa: BLE001
                    log("camera reopen failed: %s" % e)
                fail_since = time.time()      # restart the clock either way
            time.sleep(0.1)
            continue
        fail_since = None
        reopens = 0
        _cam["last_frame"] = time.time()
        h, w = frame.shape[:2]
        tube = _track_tube(detect_tube(frame), w, now=time.time())
        moist = _moisture_min()
        # ML emitter model first (the Physical-AI showcase); fall back to
        # classical detect_emitter if no model is deployed / inference fails.
        emit = detect_emitter_ml(frame, moisture=moist) if ml_available() else None
        if emit is None or not emit.get("ml_ready"):
            emit = detect_emitter(frame, moisture=moist)
        _cam.update(w=w, h=h, tube=tube, emitter=emit)
        now = time.time()

        # Dataset capture: save the RAW frame at the chosen interval (for training
        # the emitter model). Independent of drip mode; toggled from the UI.
        if _capture["on"] and (now - _capture["last"]) >= _capture["interval"]:
            _save_capture(frame)
            _capture["last"] = now

        # A run just STARTED — reset every per-run counter. These live in the loop
        # (not in _run) because the loop owns them, so on_run_start cannot clear them.
        # Without this a second run resumed the previous run's travelled distance and,
        # worse, could start mid-traverse.
        if _run["state"] == "running" and prev_run_state != "running":
            phase, lateral, travelled, traversed = "follow", 0, 0.0, 0.0
            cross_hist = []
            _track_reset()
            last_plant_at, armed, driving_since = -1e9, True, None
        prev_run_state = _run["state"]

        # Tube found/lost EDGE logging. Logging every frame would flood at frame rate,
        # but logging nothing left "the robot moved 5cm and stopped" with no trace at
        # all in the log (2026-08-15) — the stall was invisible. So: log the two
        # transitions only, and only while a follow-run is active, otherwise an idle
        # camera would chatter every time something passes in front of it.
        _following = (_run["state"] == "running"
                      and (_run["mode"] == "scan"
                           or (_run["mode"] == "drip" and phase == "follow")))
        if not _following:
            tube_seen = None                  # next run starts clean, logs its first edge
        elif tube["found"] != tube_seen:
            tube_seen = tube["found"]
            if tube_seen:
                log("tube REGAINED (x=%s px, correction=%+.2f) — driving resumes"
                    % (tube.get("tube_x"), tube.get("correction") or 0.0))
            else:
                log("tube LOST — motors stopped. Run is STILL ACTIVE and will drive "
                    "again by itself when the tube is back in view; capture keeps "
                    "running throughout. Press Stop to end the run.")

        if _run["mode"] == "scan" and _run["state"] == "running":
            # CAMERA-TAB RUN: follow the tube and collect frames. The seeder is not
            # touched at all — no indexSpool, no plantSeed, not even in dry form. It
            # also does NOT stop at emitters: the detector still annotates the live
            # view so you can see what it would find, but stopping would fill the
            # dataset with near-duplicate frames of whatever it stopped in front of.
            # Ends only when the operator presses Stop, or if the tube is lost.
            if tube["found"]:
                d = _steer(tube)
                l, r = trimmed(_BASE_PWM + d, _BASE_PWM - d)
                _drive(max(0, min(255, l)), max(0, min(255, r)), "scan", "forward")
                if driving_since:
                    travelled += (now - driving_since) * _DRIP_SPEED_MPS
                driving_since = now
            else:
                _drive_stop("scan")          # lost the tube — don't drive blind
                driving_since = None

        elif _run["mode"] == "drip" and _run["state"] == "running" and phase == "traverse":
            # BETWEEN LATERALS: drive across the rows looking for the next tube.
            # NOTE the search is NOT a workaround for a bad pivot — the turn is closed on
            # the gyro now and lands within a degree. It exists because real drip layouts
            # are not parallel or evenly spaced, so where the next lateral IS cannot be
            # computed from a heading and a row gap however accurate the heading is.
            # detect_crossing() is detect_tube with the angle filter inverted — the
            # next lateral arrives near-HORIZONTAL, which detect_tube throws away.
            cross = detect_crossing(frame)
            if driving_since:
                traversed += (now - driving_since) * _DRIP_SPEED_MPS

            # APPROACH, not a single good frame. Keep recent sightings and ask whether
            # the thing is coming towards us — a lateral we are driving at moves steadily
            # DOWN the frame, soil artefacts do not. Dropouts thin this history instead
            # of resetting a counter, which is what kept the old rule from ever firing.
            far_enough = traversed >= _TRAVERSE_MIN_M
            if cross["found"]:
                cross_hist.append((now, cross["tube_y"]))
            cross_hist = [s for s in cross_hist if now - s[0] <= _TRAVERSE_TRACK_S]
            grew = (cross_hist[-1][1] - cross_hist[0][1]) if len(cross_hist) >= 2 else 0.0
            approaching = (len(cross_hist) >= _TRAVERSE_MIN_SIGHTS
                           and grew >= _TRAVERSE_APPROACH_PX)
            arrived = cross["found"] and cross["nearness"] >= _TRAVERSE_NEAR

            # SAY WHAT WE SEE. A field run drove over two real laterals and reported
            # only "no next lateral found" — no way to tell whether the detector saw
            # nothing, or saw them and rejected them. Logged ~1/s so the next run
            # produces numbers to tune against.
            if now - last_cross_log >= 1.0:
                last_cross_log = now
                log("  traverse %.2fm | found=%s y=%s near=%.2f w=%s s=%.1f %s | "
                    "sights=%d grew=%+.0fpx%s"
                    % (traversed, cross["found"], cross["tube_y"], cross["nearness"],
                       cross["width"], cross["strength"], cross["polarity"] or "-",
                       len(cross_hist), grew,
                       "" if far_enough else "  (still inside min-traverse)"))

            if far_enough and approaching and arrived:
                _drive_stop("drip")
                turn_right = (lateral % 2 == 0)       # same way as the first turn
                pre = _save_named(frame, "lat%d_found" % (lateral + 2))
                log("next lateral found after %.2fm (nearness %.2f, y=%s, w=%s, "
                    "%.1f sigma %s) — turning on [decision frame %s]"
                    % (traversed, cross["nearness"], cross["tube_y"], cross["width"],
                       cross["strength"], cross["polarity"], pre))
                reached = _arrive_over_lateral(src)   # get the pivot axis ONTO the tube
                log("  arrived over the lateral (nearness reached %.2f, then %.2fm "
                    "of camera lookahead) — pivoting %s"
                    % (reached, _ARRIVE_EXTRA_M, "right" if turn_right else "left"))
                _track_reset()                        # new row: do not gate on the old x
                _turn_onto_tube(src, turn_right, "lat%d" % (lateral + 2))
                lateral += 1
                phase = "follow"
                travelled = 0.0
                last_plant_at = -1e9                  # first emitter of a row is never gated
                armed = True
                driving_since = None
                _emit_run()
            elif traversed >= _traverse_max():
                _drive_stop("drip")
                _run["state"] = "done"
                _run["msg"] = ("no next lateral found within %.1fm — stopped after %d "
                               "lateral(s), %d emitters"
                               % (_traverse_max(), lateral + 1, _run["planted"]))
                log(_run["msg"])
                _emit_report(_drip_runlog())
                _emit_run()
            else:
                _drive(*trimmed(_BASE_PWM, _BASE_PWM), "drip", "forward")
                driving_since = now

        elif _run["mode"] == "drip" and _run["state"] == "running":
            # "Detected" for the purpose of STOPPING means confident AND near — an
            # emitter seen at the top of the frame is metres away (see _EMIT_MIN_Y_FRAC).
            in_view  = emit["detected"] and emit["confidence"] >= _EMIT_CONF
            detected = in_view and _emitter_in_reach(emit, h)
            # Debounce on the DETECTION EDGE, not on an assumed spacing: the model is
            # what finds emitters, so "this one again" means "still in view". Re-arm
            # once it clears the frame entirely (in_view, not detected — otherwise the
            # same emitter re-arms the moment it drops below the reach line and gets
            # planted twice). _min_replant() is only a floor against double-counting one
            # emitter — it never TRIGGERS a plant, so a missed emitter simply means the
            # next one is found normally.
            if not in_view:
                armed = True

            if not tube["found"]:
                # NO TUBE IN VIEW -> STOP, whatever else is happening this frame.
                # This used to be the last `else` of an if/elif chain, so a frame with
                # an emitter detected-and-armed but inside the re-plant floor matched
                # NO branch: no drive call, no stop call, and the previous setMotors
                # stayed latched. The robot drove on with nothing in view until the
                # operator hit Stop. Stopping is now decided first, not last.
                _drive_stop("drip")
                driving_since = None
            elif detected and armed and travelled - last_plant_at >= _min_replant():
                _drive_stop("drip")
                # THE frame this stop was taken on, kept so a run can be audited after
                # the fact: was it a real emitter, and was the robot actually on top of
                # it? A confidence number in a log line cannot answer either.
                shot = _save_named(frame, "emit%d_lat%d" % (_run["planted"] + 1, lateral + 1))
                # One stop, then plant at EVERY selected arm position: [0, 90] gives
                # a 4-seed cross (0/180 then 90/270).
                for a in _drip["angles"]:
                    if _drip_rotates():
                        Bridge.call("indexSpool", int(a))
                        time.sleep(_ARM_DWELL_S)      # servo must arrive before the drop
                    if not _run["dry"]:
                        Bridge.call("plantSeed")      # both outlets: 2 seeds, 180 apart
                    else:
                        # Hold for as long as the real thing takes. With a single arm
                        # position at 0 there is no spool dwell either, so the stop was
                        # microseconds long and invisible — the operator saw the robot
                        # drive straight past 11 emitters it had correctly detected.
                        time.sleep(_PLANT_SIM_S)
                if _drip_rotates():
                    Bridge.call("indexSpool", 0)      # leave the arm flat for driving
                ey = (emit.get("position") or (0, 0))[1]
                log("  emitter %d — STOPPED at %.2fm (conf %.2f, y=%d/%d) [frame %s]%s"
                    % (_run["planted"] + 1, travelled, emit["confidence"], ey, h, shot,
                       " (dry: holding %.1fs, seeder idle)" % _PLANT_SIM_S
                       if _run["dry"] else ""))
                _run["planted"] += 1
                _run["positions"].append((0.0, round(travelled, 3)))
                armed = False
                last_plant_at = travelled
                _emit_run()
            # End of lateral is the MARKED plot length, so a long real lateral can be
            # demoed over a deliberately shorter marked plot.
            elif travelled >= _plot["l"]:
                _drive_stop("drip")
                if lateral + 1 >= max(1, int(_drip.get("laterals", 1))):
                    _run["state"] = "done"
                    _run["msg"] = ("done — %d lateral(s), %d emitters"
                                   % (lateral + 1, _run["planted"]))
                    _emit_report(_drip_runlog())
                    _emit_run()
                else:
                    # ROW CHANGE BY SEARCH, not by dead reckoning. Turn off the row,
                    # then drive across looking for the next tube. Direction alternates
                    # so the path serpentines; both turns of a pair go the same way.
                    turn_right = (lateral % 2 == 0)
                    log("lateral %d done (%d emitters) — turning %s to find the next"
                        % (lateral + 1, _run["planted"], "right" if turn_right else "left"))
                    _pivot(90, turn_right)
                    phase = "traverse"
                    traversed = 0.0
                    cross_hist = []
                    driving_since = None
                    _emit_run()
            else:
                # Tube in view, nothing to plant, row not finished: follow it.
                d = _steer(tube)
                l, r = trimmed(_BASE_PWM + d, _BASE_PWM - d)
                _drive(max(0, min(255, l)), max(0, min(255, r)), "drip", "forward")
                if driving_since:                     # integrate travel while moving
                    travelled += (now - driving_since) * _DRIP_SPEED_MPS
                driving_since = now
        else:
            driving_since = None

        # Push an annotated frame to the browser — only while the Camera tab is
        # open (recent get_vision poll) and rate-limited, so the cam still gets
        # every frame for detection while the operator sees a smooth view.
        if now - _cam["watch"] < _WATCH_TTL and now - last_push >= push_interval:
            try:
                vis = draw_overlay(frame, tube, emit)
                ok2, buf = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok2:
                    ui.send_message("cam_frame", {
                        "jpeg": base64.b64encode(buf).decode("ascii"),
                        "vision_ok": True,        # frames only flow when vision is up
                        "w": w, "h": h, "tube": tube, "emitter": emit,
                        "drip": _drip_ui_state(),
                    })
                    last_push = now
            except Exception as e:                       # noqa: BLE001
                print("cam push failed: %s" % e, flush=True)
        time.sleep(0.02)
    src.release()

def on_set_camera(client, data):
    global _cam_thread
    url = data.get("url", "").strip()
    # "Alive" is NOT enough — a livelocked loop is alive and delivering nothing, and
    # treating that as streaming is what made Connect a no-op. Require a RECENT FRAME.
    age = time.time() - _cam.get("last_frame", 0.0)
    alive = bool(_cam_thread and _cam_thread.is_alive())
    streaming = alive and age < _CAM_STALE_S
    if url == _cam["url"] and streaming:
        return                       # genuinely streaming this url — no duplicate loop
    if alive and not streaming:
        log("camera loop alive but stale (no frame for %.0fs) — restarting it" % age)
        _cam["url"] = ""             # drops the old loop out of its while condition
        time.sleep(0.3)              # let it notice before we start the replacement
    _cam["url"] = url                # a changed url makes the old loop exit (while _cam["url"]==url)
    if url and _VISION_OK:
        _cam_thread = threading.Thread(target=_cam_loop, args=(url,), daemon=True)
        _cam_thread.start()

def on_get_vision(client, data):
    _cam["watch"] = time.time()          # tab is open -> enable annotated-frame push
    ui.send_message("vision", {
        "vision_ok": _VISION_OK,
        "w": _cam["w"], "h": _cam["h"],
        "tube": _cam["tube"], "emitter": _cam["emitter"],
        "drip": _drip_ui_state(),
    })

def _drip_rotates():
    """True if the spool has to move at all — a single 0 deg position needs no rotation."""
    a = _drip["angles"]
    return len(a) > 1 or (a and a[0] != 0)


def _drip_seeds_per_emitter():
    return 2 * max(1, len(_drip["angles"]))       # 2 outlets per arm position


_gyro = {"ok": None}          # None = not probed yet


def _gyro_ready():
    """Is the MCU's gyro answering? Probed once, then cached."""
    if _gyro["ok"] is None:
        try:
            r = json.loads(_decode(Bridge.call("getGyro")))
            _gyro["ok"] = bool(r.get("ok"))
            if _gyro["ok"]:
                log("gyro ready on %s (%s), residual %.3f dps — turns are CLOSED-LOOP"
                    % (r.get("bus"), r.get("addr"), r.get("mean_dps", 0.0)))
            else:
                log("no gyro (%s) — turns fall back to the TIMED pivot"
                    % r.get("err", "?"))
        except Exception as e:                             # noqa: BLE001
            _gyro["ok"] = False
            log("gyro probe failed (%s) — turns fall back to the TIMED pivot" % e)
    return _gyro["ok"]


def _pivot_timed(deg, turn_right):
    """The original OPEN-LOOP pivot: run the motors for a calibrated time.

    Kept only as the fallback for a missing gyro, and it is the weakest step in the row
    change. Its premise is a fixed deg/s, and that premise is false: CAL["tdps"] = 45.2
    was measured on soil, while the gyro measured 68-70 deg/s for the same command on a
    hard floor. There is no single correct value, which is the whole argument for closing
    the loop — a 90 deg command here can be out by tens of degrees on the wrong surface.
    """
    p = CAL["turn_pwm"]
    left, right = (p, -p) if turn_right else (-p, p)      # right = CW
    secs = max(0.0, CAL["tstartup"] + abs(deg) / CAL["tdps"])
    _drive(*trimmed(left, right), "drip", "right" if turn_right else "left")
    time.sleep(secs)
    _drive_stop("drip")
    time.sleep(0.3)                                        # settle before believing frames


def _pivot(deg, turn_right):
    """Pivot by `deg`, closed on the GYRO when one is fitted.

    Blocking is deliberate: this runs inside the camera loop, and a ~1.5s stall between
    laterals is harmless — there is nothing to see while the robot spins.

    The MCU drives until the measured chassis rotation reaches the target (less the coast
    it has learned), so SKID NO LONGER COSTS ANGLE — it only makes the turn take longer,
    because the gyro watches the body rather than the wheels. Measured 2026-08-17 over
    four consecutive 90 deg turns: worst error 0.3 deg, and 355 deg of real rotation for
    360 deg commanded, i.e. no accumulation.

    The one failure it cannot fix is being physically STUCK — a wheel dug into soft soil
    that cannot rotate. Then the MCU times out and says so, which is the point: the timed
    pivot reported success and drove off at the wrong heading.
    """
    if not _gyro_ready():
        _pivot_timed(deg, turn_right)
        return
    try:
        r = json.loads(_decode(Bridge.call(
            "pivotDeg", int(abs(deg)), int(CAL["turn_pwm"]), 1 if turn_right else 0)))
    except Exception as e:                                 # noqa: BLE001
        log("  pivot: gyro RPC failed (%s) — falling back to the timed pivot" % e)
        _pivot_timed(deg, turn_right)
        return
    if not r.get("ok"):
        log("  pivot: %s — falling back to the timed pivot" % r.get("err", "?"))
        _pivot_timed(deg, turn_right)
        return

    timed_out = r.get("timeout") in (True, "true")
    log("  pivot %s %d deg -> achieved %.1f (err %+.1f, %d ms, avg %.1f dps, "
        "coast %.1f, judder %.1f)%s"
        % ("right" if turn_right else "left", abs(deg), r["achieved"], r["err_deg"],
           r["ms"], r.get("avg_dps", 0.0), r.get("coast_seen", 0.0),
           r.get("judder_deg", 0.0),
           "  ** TIMED OUT — robot could not rotate (wheel dug in?) **" if timed_out
           else ""))
    time.sleep(0.3)                                        # settle before believing frames


def _drip_ui_state():
    """"following"/"idle" for the camera overlay. Covers BOTH tube-following runs —
    `scan` (camera tab, collect frames) and `drip` (seed tab, plant at emitters)."""
    return ("following" if (_run["mode"] in ("scan", "drip")
                            and _run["state"] == "running") else "idle")


def _drip_readiness():
    """Camera readiness — the model IS the plant trigger, so surface its state."""
    emit = _cam.get("emitter") or {}
    tube = _cam.get("tube") or {}
    gap = max(0.05, _drip["emitter_gap"])
    return {
        "emitter_gap": _drip["emitter_gap"],
        "angles": list(_drip["angles"]),
        "laterals": _drip.get("laterals", 1),
        "rotates": _drip_rotates(),
        "seeds_per_emitter": _drip_seeds_per_emitter(),
        "cam_connected": bool(_cam["url"]),
        "detector": ("ML/Edge-Impulse" if (_VISION_OK and ml_available())
                     else "classical CV" if _VISION_OK else "unavailable"),
        "tube_found": bool(tube.get("found")),
        "emitter_conf": emit.get("confidence"),
        # only an ESTIMATE for the counter — it never gates a plant
        "expected": int(_plot["l"] / gap) + 1,
    }


def on_drip_config(client, data):
    if "laterals" in data:
        _drip["laterals"] = max(1, min(20, int(data["laterals"])))
    if "emitter_gap" in data:
        _drip["emitter_gap"] = max(0.05, float(data["emitter_gap"]))
    if "angles" in data:
        # sanitise: each is an arm position mod 180 (its pair lands at +180),
        # de-duplicated and ordered so the spool sweeps one way; never empty.
        vals = sorted({int(a) % 180 for a in (data["angles"] or [])})
        _drip["angles"] = vals or [0]
    _emit_run()

# ── Dataset capture (drive the drip line, collect the emitter training set) ──
def _save_capture(frame):
    """Save one RAW frame to disk + push a thumbnail to the UI for verification."""
    try:
        os.makedirs(_CAP_DIR, exist_ok=True)
        name = "cap_%s.jpg" % datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        cv2.imwrite(os.path.join(_CAP_DIR, name), frame)     # full-res, unannotated
        _capture["count"] += 1
        th = cv2.resize(frame, (160, 120))
        ok, buf = cv2.imencode(".jpg", th, [cv2.IMWRITE_JPEG_QUALITY, 55])
        ui.send_message("capture_saved", {
            "name": name, "count": _capture["count"],
            "thumb": base64.b64encode(buf).decode("ascii") if ok else "",
        })
    except Exception as e:                                    # noqa: BLE001
        print("capture save failed: %s" % e, flush=True)

def _save_named(frame, tag):
    """Save one frame under a decision tag; return the filename (or "")."""
    try:
        os.makedirs(_CAP_DIR, exist_ok=True)
        name = "%s_%s.jpg" % (tag, datetime.now().strftime("%H%M%S"))
        cv2.imwrite(os.path.join(_CAP_DIR, name), frame)
        return name
    except Exception as e:                                    # noqa: BLE001
        print("named save failed: %s" % e, flush=True)
        return ""


def _fresh_frame(src, budget_s=1.2):
    """Read past buffered frames and return the LIVE one.

    A pivot blocks for a second or more while the MJPEG decoder keeps buffering, so
    src.read() straight afterwards hands back the view from mid-spin — and an alignment
    check on that frame is worse than no check at all. Buffered reads return instantly;
    a read at the live edge has to wait for the camera. So: read until one waits.
    (CAP_PROP_BUFFERSIZE would be the tidy fix. It kills this stream — see camera.py.)
    """
    ok, frame = src.read()
    deadline = time.time() + budget_s
    while ok and time.time() < deadline:
        t0 = time.time()
        ok2, f2 = src.read()
        if not ok2:
            break
        frame = f2
        if time.time() - t0 > 0.04:        # this read waited on the network -> live
            break
    return ok, frame


def _tilt_str(t):
    a = t.get("angle_deg")
    return "tilt n/a" if a is None else "%+.0f deg off vertical" % a


# Turning ONTO a found lateral, closed-loop. The coarse pivot deliberately UNDERSHOOTS
# 90 so the tube is approached from one side only — detect_tube accepts up to 30 deg off
# vertical, so it is already visible at 72 — and the remainder is closed on what the
# camera sees rather than on the pivot calibration, which is the least trustworthy number
# in the row change (open-loop, surface-dependent).
#
# The nudge is a fixed short PULSE, not _pivot(small_deg): with tstartup = -0.80 the
# pivot model clamps anything under ~36 deg to zero command time, so small pivots simply
# do not happen. A closed loop does not need the step size calibrated — only that it is
# small, repeatable and in the right direction. Each nudge logs the correction before and
# after, so the field log measures the real per-nudge effect for us.
# The coarse pivot is now the FULL 90. It was 72 — a deliberate undershoot, so the tube
# would always be approached from one side and every nudge could go the same way. That
# reasoning was entirely about the open-loop pivot being unreliable; with the gyro closing
# the turn to +/-0.3 deg it is not just unnecessary but harmful, since an 18 deg
# undershoot would CREATE the misalignment the nudges then have to remove. Vision now
# confirms and trims rather than doing the bulk of the work.
_TURN_ON_COARSE = 90
# These stay PULSE-based rather than calling pivotDeg with a small angle, for a concrete
# reason: the learned coast at turn_pwm is ~8 deg, so pivotDeg cannot deliver a 5 deg nudge
# at that duty — it would release almost immediately and coast straight past. A lower PWM
# would coast less, but s_coast_deg is a single global on the MCU, so mixing duties would
# poison the estimate the 90 deg turns depend on. A vision-closed pulse needs no
# calibration at all, which is exactly why it is the right tool here.
# The pulse GROWS until the robot actually moves. 0.15s produced literally nothing on
# 2026-08-17: two consecutive alignment frames came back x=195.0, tilt=+9.8, strength
# 3.1 — different JPEGs (sensor noise), identical scene. From a standstill a short pulse
# is spent on stiction and the wheels never break away. Rather than guess a number that
# works on one surface, start small and lengthen whenever the view does not change; the
# camera says whether the last pulse did anything.
_NUDGE_PULSE_S   = 0.30
_NUDGE_PULSE_MAX = 0.75
_NUDGE_GROWTH    = 1.6
_NUDGE_TOL       = 0.80   # |correction| (+/-5 full scale) close enough to hand over
_NUDGE_MAX       = 6


def _nudge(turn_right, pulse):
    p = CAL["turn_pwm"]
    left, right = (p, -p) if turn_right else (-p, p)
    _drive(*trimmed(left, right), "drip", "right" if turn_right else "left")
    time.sleep(pulse)
    _drive_stop("drip")
    time.sleep(0.35)                       # settle before believing the next frame


# Closing the last stretch before the pivot. The camera looks AHEAD, so a crossing at
# nearness 0.6 is still well in front of the wheels — pivoting there leaves the lateral
# beside the robot rather than under it. That is what happened on 2026-08-17: the turn
# fired at nearness 0.61 and the very next frame showed no vertical tube at all, with the
# lateral still crossing at y=221. So drive the band down to the bottom of the frame
# first, then blind-creep the camera's own lookahead.
_ARRIVE_NEAR    = 0.88
_ARRIVE_EXTRA_M = 0.20   # camera bottom-edge -> pivot axis. ESTIMATE. To measure: park
                         # so a tube sits on the bottom edge of the live view, then
                         # measure tube to wheel axis on the ground.
_ARRIVE_MAX_S   = 6.0


def _creep(metres):
    """Blind forward creep of roughly this distance (open-loop, like _pivot)."""
    _drive(*trimmed(_BASE_PWM, _BASE_PWM), "drip", "forward")
    time.sleep(max(0.0, metres / max(0.01, _DRIP_SPEED_MPS)))
    _drive_stop("drip")
    time.sleep(0.3)


def _arrive_over_lateral(src):
    """Creep until the lateral is at the wheels. Returns the best nearness reached."""
    deadline = time.time() + _ARRIVE_MAX_S
    best = 0.0
    while time.time() < deadline:
        _drive(*trimmed(_BASE_PWM, _BASE_PWM), "drip", "forward")
        ok, frame = src.read()
        if not ok:
            continue
        c = detect_crossing(frame)
        if c["found"]:
            best = max(best, c["nearness"])
            if c["nearness"] >= _ARRIVE_NEAR:
                break
        elif best >= 0.75:
            break                      # swept out of the bottom of the frame: we are on it
        time.sleep(0.05)
    _drive_stop("drip")
    time.sleep(0.2)
    _creep(_ARRIVE_EXTRA_M)
    return best


def _turn_onto_tube(src, turn_right, tag):
    """Pivot onto the lateral, then centre the tube in frame before driving off.

    Still needed after the gyro: a geometrically perfect 90 deg turn does not put the robot
    ON the tube, because the lateral need not be square to the path we crossed on. The
    gyro fixes the HEADING CHANGE; only the camera can say where the tube actually is. What
    changed is the division of labour — vision now trims a good turn instead of rescuing a
    bad one.

    Every frame the decision is taken on is saved as <tag>_alignN.jpg, and the numbers
    are logged beside the filename, so a bad landing can be checked against the picture
    that caused it instead of inferred from the steering flailing seconds later.
    """
    _pivot(_TURN_ON_COARSE, turn_right)
    last = None                            # last accepted correction
    last_x = None                          # last tube_x seen, to detect "nothing moved"
    pulse = _NUDGE_PULSE_S
    for i in range(_NUDGE_MAX + 1):
        ok, frame = _fresh_frame(src)
        if not ok:
            log("  align: no frame after the turn — leaving it open-loop")
            return
        t = detect_tube(frame)
        plausible = _tube_plausible(t)
        name = _save_named(frame, "%s_align%d" % (tag, i))

        if not plausible:
            if i >= _NUDGE_MAX:
                break
            log("  align %d: no tube-shaped vertical yet (found=%s w=%s s=%.1f) — "
                "nudging %s %.2fs [%s]"
                % (i, t["found"], t["width"], t["strength"],
                   "right" if turn_right else "left", pulse, name))
            _nudge(turn_right, pulse)      # under-turned: keep coming round
            pulse = min(_NUDGE_PULSE_MAX, pulse * _NUDGE_GROWTH)
            last, last_x = None, None
            continue

        c = t["correction"]
        log("  align %d: tube x=%.0f, off %+.0f px, correction %+.2f, w=%s s=%.1f, %s [%s]"
            % (i, t["tube_x"], t["offset_px"], c, t["width"], t["strength"],
               _tilt_str(t), name))
        if abs(c) <= _NUDGE_TOL:
            log("  align: centred after %d nudge(s) — following" % i)
            return
        if last_x is not None and abs(t["tube_x"] - last_x) < 3.0:
            # The view did not change: that pulse moved nothing. Lengthen it and retry
            # rather than concluding there is no gain to be had — the old code read this
            # as "no further gain" and handed over a robot that had not turned at all.
            pulse = min(_NUDGE_PULSE_MAX, pulse * _NUDGE_GROWTH)
            log("  align: nothing moved — lengthening the nudge to %.2fs" % pulse)
        elif last is not None and (c * last < 0 or abs(c) >= abs(last) - 0.05):
            # It DID move, and moved past centre or not usefully. Stop: the follow loop
            # steers continuously while driving and does this better than pivoting in
            # place can. Hunting here would just burn the tube out of view.
            log("  align: no further gain (was %+.2f, now %+.2f) — handing to the "
                "follow loop" % (last, c))
            return
        if i >= _NUDGE_MAX:
            break
        _nudge(c > 0, pulse)               # + correction = tube is RIGHT -> turn right
        last, last_x = c, t["tube_x"]
    log("  align: not centred after %d nudges — the follow loop takes it from here"
        % _NUDGE_MAX)


def _emit_capture_status():
    ui.send_message("capture_status", {
        "on": _capture["on"], "count": _capture["count"],
        "interval": _capture["interval"], "dir": _CAP_DIR,
        "cam_connected": bool(_cam["url"]),
    })

def on_capture_start(client, data):
    _capture["interval"] = max(0.3, float(data.get("interval", 2.0)))
    _capture["last"] = 0.0            # capture the next frame immediately
    _capture["on"] = True
    os.makedirs(_CAP_DIR, exist_ok=True)
    print("capture: ON (every %.1fs) -> %s" % (_capture["interval"], _CAP_DIR), flush=True)
    _emit_capture_status()

def on_capture_stop(client, data):
    _capture["on"] = False
    print("capture: OFF (%d saved)" % _capture["count"], flush=True)
    _emit_capture_status()

def on_get_capture(client, data):
    _emit_capture_status()

def on_capture_clear(client, data):
    _capture["on"] = False
    removed = 0
    try:
        for f in os.listdir(_CAP_DIR):
            if f.startswith("cap_") and f.endswith(".jpg"):
                os.remove(os.path.join(_CAP_DIR, f)); removed += 1
    except FileNotFoundError:
        pass
    except Exception as e:                                    # noqa: BLE001
        print("capture clear failed: %s" % e, flush=True)
    _capture["count"] = 0
    print("capture: cleared %d files" % removed, flush=True)
    ui.send_message("capture_cleared", {"removed": removed})
    _emit_capture_status()


# ── Robot stats (dependency-free — read /proc directly) ─────────────────────
# Pushed to the operator console on request (client polls every ~2s). Kept
# lightweight and defensive: any read failure just yields None for that field,
# so the UI shows "—" rather than crashing.

def _mem_percent():
    try:
        info = {}
        with open("/proc/meminfo") as f:
            for line in f:
                k, _, rest = line.partition(":")
                info[k] = int(rest.strip().split()[0])   # kB
        total = info.get("MemTotal", 0)
        avail = info.get("MemAvailable", info.get("MemFree", 0))
        return round(100 * (1 - avail / total)) if total else None
    except Exception:
        return None

def _cpu_times():
    with open("/proc/stat") as f:
        vals = list(map(int, f.readline().split()[1:]))
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)   # idle + iowait
    return sum(vals), idle

try:
    _prev_cpu = _cpu_times()
except Exception:
    _prev_cpu = None

def _cpu_percent():
    global _prev_cpu
    try:
        total, idle = _cpu_times()
        if _prev_cpu is None:
            _prev_cpu = (total, idle)
            return None
        dt, di = total - _prev_cpu[0], idle - _prev_cpu[1]
        _prev_cpu = (total, idle)
        return round(100 * (1 - di / dt)) if dt > 0 else None
    except Exception:
        return None

def _uptime_str():
    try:
        with open("/proc/uptime") as f:
            secs = int(float(f.readline().split()[0]))
        h, m = secs // 3600, (secs % 3600) // 60
        return ("%dh %dm" % (h, m)) if h else ("%dm" % m)
    except Exception:
        return None

_batt_ema = {"pct": None, "v": None}       # smoothed battery (EMA)
_BATT_ALPHA = 0.2                          # new-sample weight; ~10s settle at a 2s poll

def _battery():
    """Read the 3S LiPo via the getBattery RPC (A4 divider), smoothed with an EMA.
    Battery changes slowly, so heavy smoothing kills the mid-curve %-jitter and the
    occasional ADC spike. Returns (pct, volts) or (None, None) if the RPC is absent."""
    try:
        b = json.loads(_decode(Bridge.call("getBattery")))
        pct = float(b.get("pct")); v = float(b.get("volts"))
    except Exception:                       # noqa: BLE001
        return None, None
    a = _BATT_ALPHA
    # reject a wild single-sample spike (>25% jump) so one glitch can't yank the EMA
    if _batt_ema["pct"] is not None and abs(pct - _batt_ema["pct"]) > 25:
        a *= 0.25
    _batt_ema["v"]   = v   if _batt_ema["v"]   is None else (1 - a) * _batt_ema["v"]   + a * v
    _batt_ema["pct"] = pct if _batt_ema["pct"] is None else (1 - a) * _batt_ema["pct"] + a * pct
    return int(round(_batt_ema["pct"])), round(_batt_ema["v"], 2)

# ── Plot seeding (Act 2): mark a plot, plan a serpentine, drive it ──────────

# The 4 corner marks are a MOCK: the operator walks the plot and taps each corner,
# which is the demo gesture for "this is the land". There is no absolute position
# sensor (no GPS, no encoders), so the marks are presentational — the geometry the
# robot actually drives comes from the entered width/length. Corner 1 is where the
# robot starts, heading along the first edge.
_plot = {
    "land": "plain",
    "corners": [], "w": 5.0, "l": 5.0,
    "row_gap": 0.70, "seed_gap": 0.40, "seeds_per_spot": 1,   # laterals measured 2026-08-17
}

# ONE run for both land types. The robot does the same job either way — drive,
# stop, plant, log, report — only the PLANT TRIGGER differs: a planned grid
# position (plain) or an emitter the model saw (drip). So state, controls, dry
# run, progress and the report are shared; only the config cards differ.
_run = {
    "mode": "plain",            # plain | drip | scan
    "state": "idle",            # idle | running | paused | done | stopped | error
    "planted": 0, "total": 0, "msg": "", "dry": True,
    "positions": [],            # where seeds actually went, for the report
}
_plot_thread = None
# `scan` is the CAMERA tab's run: follow the tube and collect training frames, with the
# seeder never touched. Deliberately separate from `drip` rather than a flag on it —
# seeding lives on the Seed tab, and a data-collection run must not be one wrong click
# away from firing a solenoid. It also does NOT stop at emitters: standing still would
# just fill the dataset with near-duplicate frames of the same emitter.
_scan_owns_capture = False      # did scan turn capture on? then scan turns it back off

# ── MCU bridge health ───────────────────────────────────────────────────────
# A dead RouterBridge is INVISIBLE from the browser: the web UI keeps serving, keeps
# accepting clicks and keeps logging them, while every Bridge.call raises and nothing
# reaches the MCU. On 2026-08-15 that cost a whole field session — the operator
# pressed Shutdown (deliberately), the board halted and auto-restarted, and from then
# on Follow/Stop/capture all looked like dead buttons. The battery logger already polls
# getDiag every 2s, so it doubles as the health monitor: it is the one thing guaranteed
# to touch the bridge on a timer whatever else the robot is doing.
_bridge = {"ok": True, "since": 0.0, "err": ""}


def _set_bridge(ok, err=""):
    """Record bridge health; log + push only on a CHANGE, not every poll."""
    if _bridge["ok"] != ok:
        _bridge.update(ok=ok, since=time.time(), err=str(err)[:120])
        log("BRIDGE %s" % ("UP — MCU link restored" if ok else
                           "DOWN — no MCU link (%s). Motors/seeder/sensors are all "
                           "unreachable until this recovers." % (err or "?")))
    else:
        _bridge["ok"] = ok


class _Abort(Exception):
    """Raised inside the run thread when the operator stops the run."""


def _plot_cfg():
    return SeedPlan(plot_w_m=_plot["w"], plot_l_m=_plot["l"],
                    row_gap_m=_plot["row_gap"], seed_gap_m=_plot["seed_gap"],
                    seeds_per_spot=int(_plot["seeds_per_spot"]),
                    speed_mps=CAL["speed"], crop="groundnut")


def _plot_preview():
    """Planned spots + summary for the UI overlay (no driving)."""
    if not _PLOT_OK:
        return {"planned": [], "summary": {}, "est_s": 0}
    cfg = _plot_cfg()
    path = plan_boustrophedon(cfg)
    summary = plan_summary(cfg)
    # rough time: each hop is startup + gap/speed, plus ~1.3s per plant (punch+drop)
    hop = CAL["startup"] + _plot["seed_gap"] / CAL["speed"]
    plant_s = 0.0 if _run["dry"] else 1.3 * int(_plot["seeds_per_spot"])
    turns = max(0, summary.get("rows", 0) - 1) * 2
    est = len(path) * (hop + plant_s) + turns * (CAL["tstartup"] + 90 / CAL["tdps"])
    return {"planned": [[w.x, w.y] for w in path], "summary": summary,
            "est_s": int(max(0, est))}


def _emit_plot(preview=False):
    msg = {k: _plot[k] for k in
           ("land", "corners", "w", "l", "row_gap", "seed_gap", "seeds_per_spot")}
    msg.update({k: _run[k] for k in ("state", "planted", "total", "msg", "dry")})
    msg["spot"] = _run["planted"]                  # progress dots on the plan overlay
    if preview:
        msg.update(_plot_preview())
    ui.send_message("plot", msg)


def _emit_run():
    """Shared run status for both land types (state, progress, drip readiness)."""
    ui.send_message("run", {**{k: _run[k] for k in
                               ("mode", "state", "planted", "total", "msg", "dry")},
                            **_drip_readiness()})
    _emit_plot()                                   # keep the plan overlay in step


class _ProgressRobot(BridgeRobot):
    """BridgeRobot that reports progress and can be stopped mid-run.

    execute() only talks to the robot, so overriding its methods is the hook for
    both progress and abort — no changes needed in farmos/executor.py.
    """

    def _gate(self):
        while _run["state"] == "paused":
            time.sleep(0.2)
        if _run["state"] != "running":
            raise _Abort()

    def forward(self, distance_m):
        self._gate()
        super().forward(distance_m)

    def turn_to(self, heading_deg):
        self._gate()
        super().turn_to(heading_deg)

    def plant(self):
        self._gate()
        super().plant()
        _run["planted"] += 1
        _emit_run()


def _plot_loop():
    cfg = _plot_cfg()
    path = plan_boustrophedon(cfg)
    _run["total"] = len(path) * int(_plot["seeds_per_spot"])
    _run["planted"] = 0
    robot = _ProgressRobot(
        speed_mps=CAL["speed"], startup_s=CAL["startup"], pwm=int(CAL["pwm"]),
        left_trim=CAL["ltrim"], right_trim=CAL["rtrim"],
        turn_pwm=int(CAL["turn_pwm"]), turn_deg_per_s=CAL["tdps"],
        turn_startup_s=CAL["tstartup"], turn_ramp_s=CAL["tramp"],
        plant_enabled=not _run["dry"], batt_comp=False)
    log("plain run: %d spots, %s" % (len(path), "DRY (no seeder)" if _run["dry"]
                                     else "seeder ARMED"))
    _emit_run()
    try:
        runlog = execute(cfg, path, robot)
        _run["state"] = "done"
        _run["msg"] = "finished — %d spots" % len(runlog.executed)
        _emit_report(runlog)
    except _Abort:
        _run["msg"] = "stopped by operator"
    except Exception as e:                       # noqa: BLE001
        _run["state"] = "error"
        _run["msg"] = str(e)
        log("plain run FAILED: %s" % e)
    finally:
        try:
            _drive_stop("plot")
        except Exception:                        # noqa: BLE001
            pass
        if robot.warnings:                       # MCU disagreed with what we sent
            _run["msg"] += " · %d diag warning(s)" % len(robot.warnings)
            for w in robot.warnings:
                log("run diag: %s" % w)
        _emit_run()


def _emit_report(runlog):
    """Shared Act-4 report for both land types."""
    try:
        ui.send_message("run_report", {"svg": render_svg(runlog),
                                       "stats": runlog.stats})
    except Exception as e:                       # noqa: BLE001
        log("report render failed: %s" % e)


def _drip_runlog():
    """Build a RunLog from the emitters we actually planted, so drip gets a report too.

    There is no planned grid here — the model discovers the emitters — so `planned`
    is empty and render_svg simply draws no planned dots.
    """
    pts = list(_run["positions"])
    gaps = [abs(pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1)]
    mean_gap = round(sum(gaps) / len(gaps), 4) if gaps else 0.0
    per = _drip_seeds_per_emitter()
    return RunLog(
        config=_plot_cfg().to_dict(),
        planned=[],
        executed=[(round(x, 4), round(y, 4)) for x, y in pts],
        summary={"rows": 1, "seeds_per_row": len(pts),
                 "spots": len(pts), "seeds_total": len(pts) * per},
        stats={"distance_m": round(pts[-1][1], 3) if pts else 0.0,
               "est_run_time_s": 0.0,
               "planned_spacing": {"mean_gap_m": 0.0, "min_gap_m": 0.0, "max_gap_m": 0.0},
               "executed_spacing": {"mean_gap_m": mean_gap,
                                    "min_gap_m": round(min(gaps), 4) if gaps else 0.0,
                                    "max_gap_m": round(max(gaps), 4) if gaps else 0.0},
               "max_position_error_m": 0.0,       # no planned grid to compare against
               "trigger": "emitter detection (%s)" % _drip_readiness()["detector"]},
        crop=_plot_cfg().crop)


def on_plot_mark(client, data):
    """Mock corner mark — the operator walks the plot and taps each corner."""
    if len(_plot["corners"]) < 4:
        _plot["corners"].append(int(data.get("corner", len(_plot["corners"]) + 1)))
    _plot["msg"] = ("plot marked — corner 1 is the start"
                    if len(_plot["corners"]) == 4 else
                    "marked %d of 4 corners" % len(_plot["corners"]))
    _emit_plot(preview=True)


def on_plot_clear(client, data):
    _plot["corners"] = []
    _plot["msg"] = ""
    _emit_plot(preview=True)


def on_plot_config(client, data):
    for k, cast in (("w", float), ("l", float), ("row_gap", float),
                    ("seed_gap", float), ("seeds_per_spot", int)):
        if k in data:
            _plot[k] = cast(data[k])
    if "dry" in data:
        _run["dry"] = bool(data["dry"])
    if "land" in data:
        _plot["land"] = data["land"]
        _run["mode"] = data["land"]     # same run, different trigger
    _emit_plot(preview=True)
    _emit_run()


# ── Shared run controls (both land types) ──────────────────────────────────
def on_run_start(client, data):
    """Start a run. Same entry point for both land types; only the trigger differs."""
    global _plot_thread
    if "dry" in data:
        _run["dry"] = bool(data["dry"])
    if "mode" in data:
        _run["mode"] = data["mode"]
    global _scan_owns_capture
    if _run["state"] in ("running", "paused"):
        # Used to return silently, which is indistinguishable from a broken button —
        # and a tube-lost run sits in "running" while parked, so this fires often.
        _run["msg"] = ("already running (%s) — press Stop first" % _run["mode"]
                       if _run["state"] == "running" else "paused — press Start to resume")
        return _emit_run()
    # scan needs no plot: it's a data-collection drive down whatever tube is in front of
    # the camera, not a run over a marked piece of land.
    if _run["mode"] != "scan" and len(_plot["corners"]) < 4:
        _run["msg"] = "mark all 4 corners first"
        return _emit_run()

    _run.update(planted=0, total=0, msg="", positions=[])
    if _run["mode"] == "scan":
        if not (_VISION_OK and _cam["url"]):
            _run["msg"] = "connect the camera first — scan follows the tube by sight"
            return _emit_run()
        # Turn capture on for the operator: the whole point of the run is the frames.
        # Remember whether WE turned it on, so stopping the scan doesn't switch off a
        # capture the operator had already started for their own reasons.
        if not _capture["on"]:
            _capture["on"] = True
            _capture["last"] = 0.0            # first frame immediately
            _scan_owns_capture = True
            os.makedirs(_CAP_DIR, exist_ok=True)
            _emit_capture_status()
            # SAY SO. This used to happen silently, and the matching silent switch-off
            # in on_run_stop made capture look like it was toggling itself (2026-08-15).
            log("scan: capture turned ON automatically (scan started it, "
                "so Stop will turn it off again)")
        _run["state"] = "running"
        log("scan run: follow tube + capture every %.1fs -> %s. SEEDER NOT USED."
            % (_capture["interval"], _CAP_DIR))
    elif _run["mode"] == "drip":
        if not (_VISION_OK and _cam["url"]):
            _run["msg"] = "connect the camera first — the emitter model is the trigger"
            return _emit_run()
        # the camera loop does the driving; it watches _run["state"]
        _run["state"] = "running"
        log("drip run: arm positions %s (%d seeds/emitter), %s" % (
            ", ".join("%d/%d" % (a, a + 180) for a in _drip["angles"]),
            _drip_seeds_per_emitter(),
            "DRY (no seeder)" if _run["dry"] else "seeder ARMED"))
    else:
        if not _PLOT_OK:
            _run["msg"] = "farmos planner unavailable"
            return _emit_run()
        if _plot_thread and _plot_thread.is_alive():
            return
        _run["state"] = "running"
        _plot_thread = threading.Thread(target=_plot_loop, daemon=True)
        _plot_thread.start()
    _emit_run()


def on_run_pause(client, data):
    """Plain land pauses between moves. Drip has no pause — stop it instead."""
    if _run["mode"] == "drip":
        return on_run_stop(client, data)
    if _run["state"] == "running":
        _run["state"] = "paused"
    elif _run["state"] == "paused":
        _run["state"] = "running"
    _emit_run()


def on_run_stop(client, data):
    global _scan_owns_capture
    if _run["state"] in ("running", "paused"):
        _run["state"] = "stopped"        # plain: _gate() raises _Abort; drip/scan: loop exits
        if _run["mode"] == "drip" and _run["positions"]:
            _emit_report(_drip_runlog())
        if _run["mode"] == "scan":
            _run["msg"] = "scan stopped — %d frames captured" % _capture["count"]
        else:
            _run["msg"] = "stopped by operator"
    if _scan_owns_capture:               # only undo the capture WE started
        _capture["on"] = False
        _scan_owns_capture = False
        log("scan: capture turned OFF automatically (%d frames total). Scan had turned "
            "it on; use the Capture card's Start to keep collecting while parked."
            % _capture["count"])
        _emit_capture_status()
    _drive_stop("run")
    _emit_run()


def on_get_plot(client, data):
    _emit_plot(preview=True)
    _emit_run()


def on_get_diag(client, data):
    """One end-to-end trace for the Diag tab: our last command + the MCU's view.

    `mcu` is None on firmware without getDiag; `amps` is absent unless the IBT-2
    IS pins are wired and CURRENT_SENSE is enabled in the sketch.
    """
    mcu, err = None, None
    try:
        mcu = json.loads(_decode(Bridge.call("getDiag")))
    except Exception as e:                       # noqa: BLE001
        err = str(e)
    sent = dict(_last_cmd)
    ui.send_message("diag", {
        "ui":     sent,
        "age_ms": int((time.time() - sent["at"]) * 1000) if sent["at"] else None,
        "mcu":    mcu,
        "error":  err,
        "trim":   [LEFT_TRIM, RIGHT_TRIM],
    })


# ── Battery / A4 logger ─────────────────────────────────────────────────────
# The A4 divider reading has been intermittently low and unstable (see
# docs/farm-os/drive-precision.md and the firmware's BATT_PIN notes). An
# intermittent fault can't be caught by watching a terminal, so log it
# continuously with the motor state alongside — if the dips line up with driving
# it's a ground/noise problem, and if they're random it's a contact problem.
# Written under the app dir, which is mounted to ~/motor-control on the host, so
# it survives container restarts and can be pulled with scp.
_BATT_CSV = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "battery.csv"))
_BATT_CSV_MAX = 2_000_000          # ~2MB, then start a fresh file (keep one .1 backup)


def _batt_log_loop(period_s=2.0):
    header = "utc,raw,volts,pct,src,left,right,run_state\n"
    try:
        if not os.path.exists(_BATT_CSV) or os.path.getsize(_BATT_CSV) == 0:
            with open(_BATT_CSV, "w") as f:
                f.write(header)
    except OSError as e:                              # noqa: BLE001
        log("battery log disabled (%s)" % e)
        return
    while True:
        time.sleep(period_s)
        try:
            d = json.loads(_decode(Bridge.call("getDiag")))
            b = d.get("batt", {})
            row = "%s,%.1f,%.2f,%s,%s,%s,%s,%s\n" % (
                datetime.now(timezone.utc).isoformat(timespec="seconds"),
                b.get("raw", 0.0), b.get("volts", 0.0),
                _battery_pct(b.get("volts")),
                _last_cmd.get("src") or "", _last_cmd.get("left"),
                _last_cmd.get("right"), _run["state"])
            with open(_BATT_CSV, "a") as f:
                f.write(row)
            if os.path.getsize(_BATT_CSV) > _BATT_CSV_MAX:
                os.replace(_BATT_CSV, _BATT_CSV + ".1")
                with open(_BATT_CSV, "w") as f:
                    f.write(header)
            _set_bridge(True)                         # getDiag answered -> link is alive
        except Exception as e:                        # noqa: BLE001
            _set_bridge(False, e)                     # ...and this is how we learn it died
        try:
            ui.send_message("bridge", dict(_bridge))  # cheap; a fresh tab syncs in <=2s
        except Exception:                             # noqa: BLE001
            pass                                      # never let logging kill the app


def _battery_pct(volts):
    """Same per-cell LiPo curve the firmware uses, for a self-contained CSV."""
    if not volts:
        return ""
    v = volts / 3.0
    table = [(3.30, 0), (3.40, 3), (3.55, 8), (3.65, 15), (3.70, 25), (3.75, 35),
             (3.80, 45), (3.85, 55), (3.90, 65), (4.00, 80), (4.10, 90), (4.20, 100)]
    if v <= table[0][0]:
        return 0
    if v >= table[-1][0]:
        return 100
    for i in range(1, len(table)):
        if v < table[i][0]:
            v0, p0 = table[i - 1]
            v1, p1 = table[i]
            return int(p0 + (v - v0) / (v1 - v0) * (p1 - p0) + 0.5)
    return 100


def on_get_stats(client, data):
    batt_pct, batt_v = _battery()
    ui.send_message("stats", {
        "battery":   batt_pct,             # % from the getBattery RPC (None = no divider/RPC)
        "battery_v": batt_v,               # pack volts, for the tooltip + low-batt warning
        "ram":     _mem_percent(),
        "cpu":     _cpu_percent(),
        "uptime":  _uptime_str(),
        "speed":   round(_speed / 2.55),   # echo current speed % so the UI stays in sync
    })


ui.on_message("motor_cmd",     logged("motor_cmd",     on_motor_cmd))
ui.on_message("motor_stop",    logged("motor_stop",    on_motor_stop))
ui.on_message("set_speed",     logged("set_speed",     on_set_speed))
ui.on_message("shutdown",      logged("shutdown",      on_shutdown))
ui.on_message("reboot",        logged("reboot",        on_reboot))
ui.on_message("connect_wifi",  logged("connect_wifi",  on_connect_wifi))
ui.on_message("start_hotspot", logged("start_hotspot", on_start_hotspot))
ui.on_message("plant_once",    logged("plant_once",    on_plant_once))
ui.on_message("soil_sample",   logged("soil_sample",   on_soil_sample))
ui.on_message("survey_start",  logged("survey_start",  on_survey_start))
ui.on_message("survey_stop",   logged("survey_stop",   on_survey_stop))
ui.on_message("set_camera",    logged("set_camera",    on_set_camera))
ui.on_message("drip_config",   logged("drip_config",   on_drip_config))
ui.on_message("capture_start", logged("capture_start", on_capture_start))
ui.on_message("capture_stop",  logged("capture_stop",  on_capture_stop))
ui.on_message("capture_clear", logged("capture_clear", on_capture_clear))
ui.on_message("get_capture",   on_get_capture)
ui.on_message("get_soil",      on_get_soil)    # polled only while Soil tab is open
ui.on_message("get_vision",    on_get_vision)  # polled only while Camera tab is open
ui.on_message("get_stats",     on_get_stats)   # polled by the UI; unlogged (noisy)
ui.on_message("get_diag",      on_get_diag)    # polled only while Diag tab is open
ui.on_message("plot_mark",     logged("plot_mark",   on_plot_mark))
ui.on_message("plot_clear",    logged("plot_clear",  on_plot_clear))
ui.on_message("plot_config",   logged("plot_config", on_plot_config))
ui.on_message("run_start",     logged("run_start",   on_run_start))
ui.on_message("run_pause",     logged("run_pause",   on_run_pause))
ui.on_message("run_stop",      logged("run_stop",    on_run_stop))
ui.on_message("get_plot",      on_get_plot)

# start the battery/A4 logger (daemon: dies with the app, never blocks shutdown)
threading.Thread(target=_batt_log_loop, daemon=True).start()

App.run()
