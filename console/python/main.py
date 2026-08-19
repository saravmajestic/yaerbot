import base64
import json
import math
import os
import socket
import sqlite3
import struct
import subprocess
import threading
import time
import traceback
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
    from vision.camera import FrameBus
    from vision.emitter_worker import EmitterWorker
    from vision.odometry import FlowOdometer
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


# THE OPERATOR LOG MUST SURVIVE A RESTART. `docker logs` is the only record of what a run did,
# and it is volatile: on 2026-08-18 an app restart three minutes after a traverse run destroyed
# the only evidence of why the robot stopped, and the run could not be diagnosed at all. Frames
# survive because _CAP_DIR is bind-mounted to the host; the log did not.
#
# So tee every log line to a file in the app directory, which is the same bind-mounted volume the
# captures and battery CSV already use. Rotates like the battery CSV — one 4MB file plus one .1
# backup, which at a few hundred bytes a line is many hours of runs.
#
# Deliberately best-effort: a full or read-only disk must never stop the robot logging to stdout,
# so every failure here is swallowed after one complaint.
_LOG_FILE = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "console.log"))
_LOG_MAX = 4_000_000
_log_state = {"warned": False}


def log(msg):
    line = "[%s] %s" % (time.strftime("%H:%M:%S"), msg)
    print(line, flush=True)
    try:
        with open(_LOG_FILE, "a") as f:
            f.write("%s %s\n" % (datetime.now().strftime("%Y-%m-%d"), line))
        if os.path.getsize(_LOG_FILE) > _LOG_MAX:
            os.replace(_LOG_FILE, _LOG_FILE + ".1")
    except Exception as e:                                # noqa: BLE001
        if not _log_state["warned"]:
            _log_state["warned"] = True
            print("persistent log disabled (%s) — stdout only" % e, flush=True)


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
    # PLOT MODE NOW DRIVES AT CREEP DUTY, matching drip seeding. Changed 2026-08-18 on request:
    # plain-land and drip runs plant on the same ground with the same seeder, and running them at
    # different speeds meant two separate calibrations to keep honest — this project has twice
    # been burnt by a calibration outliving the conditions it was taken under.
    #
    # Both pairs are MEASURED with field_test.py, two runs each so startup dead-time is separated
    # from steady speed rather than smeared into it:
    #     PWM  55:  fwd 55 30 -> 5.00m,  fwd 55 10 -> 1.70m  => speed 0.165  startup -0.303
    #     PWM 180:  fwd 180 8 -> 5.00m,  fwd 180 3 -> 1.80m  => speed 0.640  startup +0.187
    # The old stored pair was 0.628/0.099, i.e. the speed was only 1.9% out — kept above so the
    # 180 calibration is not lost if plot mode is ever moved back to it.
    #
    # THE TRADE: a 40cm hop takes 2.12s instead of 0.81s, so a 5m row is 30s rather than 8s and
    # the 24-spot plot takes roughly 2 minutes of driving instead of 30 seconds. Slower is also
    # MORE accurate — less overshoot per stop — but PWM 55 is near the deadband, so watch for
    # stiction on rough ground. If it stalls or crawls unevenly, that is the reason.
    "pwm": 55, "speed": 0.165, "startup": -0.303,
    "ltrim": 0.83, "rtrim": 1.00,
    "turn_pwm": 120, "tdps": 45.2, "tstartup": -0.80, "tramp": 0.0,
    # MEASURED 2026-08-18: `fwd 55 10 0.83` covered 1.70m in 10s.
    # Stored WITH the duty it was measured at, because this project has twice been burnt by
    # a calibration silently outliving the conditions it was taken under. If _BASE_PWM is
    # changed and this is not re-measured, the mismatch is logged at startup rather than
    # quietly corrupting every distance in the run.
    # MEASURED 2026-08-18 with field_test.py at PWM 55, two runs solved together so the startup
    # dead-time is separated from the steady speed rather than smeared into it:
    #     fwd 55 30 0.83 -> 5.00m     fwd 55 10 0.83 -> 1.70m
    #     => speed = 0.165 m/s, startup = -0.30s
    # The previous 0.170 was an observation, not a measurement, and was only 3% out — so the
    # speed constant was NOT the cause of the old "commanded 5m, drove 7m" overrun. That run was
    # at PWM 77 against a constant of 0.161 while the true speed was ~0.225 (ratio 0.72, which is
    # exactly the overrun seen), and the PWM change already fixed it.
    "creep": 0.165, "creep_pwm": 55,
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
#
# RE-EXPRESSED IN DISTANCE, 2026-08-18. The old rule was "4 sightings within 2.5s, y grown
# 25px". At 1.4fps a 2.5s window held ~3.5 frames, so "4 sightings" demanded essentially
# EVERY frame agree — a strong test. At 30fps the same window holds 75 frames, so it asks for
# a 5% hit rate, and 25px of growth over 42cm of driving is 2.3cm of approach, which sensor
# noise clears on its own. The gate that decides the robot turns onto a row had become close
# to free to satisfy, and it is not recoverable by a later frame: turn onto the wrong thing
# and the run is over.
#
# So: a window measured in METRES DRIVEN, sightings as a FRACTION of the frames in it, and
# the growth checked against what the geometry says it should be. A lateral the robot is
# driving at sweeps down the frame at _PX_PER_M_DEPTH px per metre; soil artefacts do not
# advance with distance at all. Requiring a quarter of the geometric expectation is loose
# enough for a noisy band and still rejects anything static.
_TRAVERSE_WINDOW_M       = 0.35   # trailing distance that counts as "recent"
_TRAVERSE_MIN_SIGHT_FRAC = 0.35   # of the frames in that window
_TRAVERSE_MIN_SIGHTS     = 3      # absolute floor, so 1-of-2 frames cannot pass on fraction
_TRAVERSE_APPROACH_FRAC  = 0.25   # of the growth the geometry predicts for the distance
_TRAVERSE_APPROACH_PX_MIN = 6     # absolute floor, kept as a guard (see below)
# MINIMUM GROUND OVER WHICH THE BAND MUST HAVE BEEN WATCHED before approach is judged at all.
# Without this the scaled requirement collapses to its px floor over a very short span, and a
# unit test caught the consequence: three sightings 3mm apart with 6px of sensor wobble
# satisfied "grew >= need_px" exactly and the latch fired. The answer is not a bigger px floor
# but refusing to answer — 3mm of driving is not evidence about anything. At 5cm the geometry
# predicts 54px of sweep, so the requirement becomes 14px and noise cannot reach it.
_TRAVERSE_MIN_SPAN_M     = 0.05
#
# NOTE the requirement GROWS with the sighting span and is deliberately NOT capped. Capping it
# would make a longer observation an easier test, which is backwards: the longer we have been
# watching something, the more it must have moved to still be a lateral we are driving at.
# Sanity check on the numbers — we latch at nearness 0.60, i.e. y=144, so a band tracked from
# the top of frame gives a span of about 144/1090 = 0.13m and a requirement of 36px against
# 144px of real growth. Comfortable margin for a real lateral; a static artefact grows 0.
_ARM_DWELL_S  = 0.6
# plantSeed on the MCU is drop(500) + settle(100) + punch(500) + fill(400) = 1.5s.
# A dry run sleeps the same so its timing matches a real run.
_PLANT_SIM_S  = 2.0      # let the spool servo reach the angle before dropping
_MIN_REPLANT_M = 0.08    # absolute floor; see _min_replant() for the run-time value

# HOW CLOSE TWO DETECTIONS MUST BE TO COUNT AS THE SAME EMITTER.
#
# THE FAULT THIS FIXES, measured on the 07:18 run of 2026-08-19. The retrained model detected an
# emitter on 96 of 96 frames in one window — recall is no longer the problem — yet only 7 stops
# happened over a row holding about 13 emitter positions. Every other one, and the reason is
# geometric rather than a detection failure:
#
#     punch tip sits 0.39 m behind the NEAREST visible ground
#     emitters are         0.40 m apart
#
# "See an emitter, then creep it under the punch" therefore consumes AT LEAST one whole spacing
# while the camera is not being consulted. The logged creeps were 0.45, 0.46, 0.51, 0.46, 0.47,
# 0.47, 0.43 m — every one longer than the spacing — and the gaps between stops came out at 1.6x
# to 2.8x the spacing. The next emitter was already under or behind the robot by the time it looked
# again.
#
# CAPPING THE CREEP WAS THE OBVIOUS FIX AND IT IS NOT GOOD ENOUGH. Requirement is +/-5 cm placement,
# and the punch is 0.39 m behind the nearest visible ground, so any creep short of that leaves the
# seed short by the difference — capping at 0.25 m would miss by 14 cm.
#
# So the robot must cover the full distance, and the ONLY way to do that without driving past the
# next emitter is to keep watching while it drives. That is the conveyor below: every detection is
# converted to a TARGET DISTANCE, targets are queued, and the punch fires when the robot reaches
# each one. Several emitters are in flight at once, which is exactly the situation a 0.40 m spacing
# and a 0.39 m punch offset forces.
#
# WHY TARGETS DEDUPLICATE THEMSELVES, which is what makes the count right: an emitter seen at
# distance `gap` when the robot is at `travelled` has target `travelled + gap`. Seen again a frame
# later, `travelled` has grown by the same amount `gap` has shrunk — so the SAME physical emitter
# yields the SAME target every time it is observed, whatever row it is in. Two detections belong to
# one emitter when their targets agree.
#
# 0.15 m: comfortably wider than the jitter in that estimate (interpolation error plus a frame or two
# of travel) and comfortably narrower than the 0.40 m spacing, so two real emitters can never merge.
_EMIT_DEDUPE_M = 0.15
# Refuse to fire twice within this, whatever the queue says. Belt-and-braces only; the dedupe above
# is the real mechanism.
_EMIT_MIN_GAP_M = 0.12


def _min_replant():
    """Distance that must pass before the NEXT emitter can be planted.

    A SAFETY FLOOR ONLY. Deduplication is done by target distance in _emit_queue (see there); this
    just refuses two punches absurdly close together. The `armed` edge-detect that used to do the
    debouncing is gone: it keyed off whether an emitter was still VISIBLE, and with the camera's view
    (0.43 m) wider than the emitter spacing (0.40 m) there is essentially always one in frame, so it
    never re-armed and a 5 m lateral planted once.

    Still not a trigger — the model decides where emitters are, and a missed one just
    means the next is found normally. This only stops ONE emitter being counted twice,
    which the 03:33 run did: stops at 2.73m and 2.84m, 11cm apart on a line whose
    emitters sit 40cm apart, because the emitter left the frame during a tube dropout
    and re-armed on the way back in. Half the operator's spacing is comfortably below
    any real gap and comfortably above that.
    """
    gap = float(_drip.get("emitter_gap") or 0.0)
    return max(_MIN_REPLANT_M, min(_EMIT_MIN_GAP_M, 0.5 * gap) if gap else _EMIT_MIN_GAP_M)


def _traverse_track(hist, traversed, cross):
    """Add this frame to the crossing history and decide whether one is APPROACHING.

    Pure and side-effect free (it returns a new history) specifically so the latch that
    decides the robot turns onto a row can be unit-tested. It could not be before: it lived
    inline in the camera loop, which needs a board, so the only way to exercise it was a
    field run — and a field run is exactly where a wrong answer is expensive.

    Returns (history, info).

    EVERY frame goes into the history, sighting or not, so the sighting FRACTION is
    measurable. The old history kept only sightings, which is why 4-of-75 frames looked
    identical to 4-of-4 and the test silently weakened by 20x when the camera got faster.
    """
    hist = list(hist)
    hist.append((traversed, cross["tube_y"] if cross["found"] else None))
    hist = [s for s in hist if traversed - s[0] <= _TRAVERSE_WINDOW_M]
    sights = [s for s in hist if s[1] is not None]
    frac = len(sights) / float(len(hist)) if hist else 0.0
    span_m = (sights[-1][0] - sights[0][0]) if len(sights) >= 2 else 0.0
    grew = (sights[-1][1] - sights[0][1]) if len(sights) >= 2 else 0.0
    # What the geometry says a real approaching lateral must have moved over span_m. A band
    # that is really coming towards the robot sweeps down the frame at _PX_PER_M_DEPTH px per
    # metre driven; a shadow or a straw does not advance with distance at all.
    need_px = max(_TRAVERSE_APPROACH_PX_MIN,
                  _TRAVERSE_APPROACH_FRAC * span_m * _PX_PER_M_DEPTH)
    approaching = (len(sights) >= _TRAVERSE_MIN_SIGHTS
                   and span_m >= _TRAVERSE_MIN_SPAN_M
                   and frac >= _TRAVERSE_MIN_SIGHT_FRAC
                   and grew >= need_px)
    return hist, {"approaching": approaching, "sights": len(sights),
                  "frames": len(hist), "frac": frac, "grew": grew,
                  "span_m": span_m, "need_px": need_px}


def _traverse_max():
    """How far to search for the next lateral before giving up.

    Was a flat 4.0m, which at the current speed estimate is nearly 6m of real ground —
    the robot drives that far with nothing in view, which reads as a runaway. The
    operator has already told us how far apart the rows are, so allow three of them.
    """
    return max(1.5, 3.0 * float(_plot.get("row_gap") or 0.7))
# Emitter-model rate limit. See the camera loop for why this is not once per frame.
# How old a frame may be and still be acted on. At 0.22 m/s the robot covers ~4cm in
# 200ms; beyond that the view no longer describes where it is. Before the frame bus there
# was no timestamp at all, so a 2s-old buffered frame was indistinguishable from a live one
# and got steered on — the silent half of the stale-frame problem.
_FRAME_MAX_AGE_S = 0.25   # unused: superseded by bus.stale_after(), which MEASURES the
                          # camera's real frame interval instead of assuming one
# INFERENCE RUNS ON ITS OWN THREAD (vision/emitter_worker.py), so this number no longer means
# what its predecessor _EMIT_TTL_S meant, and the difference is the whole point of Phase 3b.
#
# _EMIT_TTL_S was a STALENESS BUDGET that the control loop paid for: at every expiry the
# steering loop stopped for 80ms to run a model steering does not use, and in between it reused
# an old answer while 9 fresher frames sat unexamined on the bus.
#
# This is a THROTTLE ON THE ACCELERATOR, which the control loop does not pay for at all. Left
# at 0 the worker would run flat out and keep the inference container continuously busy for a
# result the robot only needs while an emitter is in the reach band. Sizing it: that band is
# the bottom 45% of the frame, ~108px, which at the measured 1090 px/m is ~10cm of ground and
# ~0.6s of travel — so 0.15s gives four independent looks inside the window that matters.
#
# 0.15 -> 0.25 after the first field run MEASURED the cost. The log reported inference at
# 85-96ms (I had estimated 70-90) and the control loop at 18-20 fps against a camera delivering
# 29.7 — so roughly a third of the frames were going unprocessed. At 0.15s that is ~60% duty on
# the worker, and although it runs on its own thread it still contends for the GIL and shares the
# process with the JPEG encode. Threading removed the BLOCKING, not the CPU cost.
#
# 0.25s is ~36% duty. It costs emitter looks inside the reach band — ~2.4 per emitter instead of
# ~4 — which is why _EMIT_CONF was lowered to 0.80 in the same breath, and why the fresh
# re-confirm at the plant stop exists to recover the precision. Steering needs the frames more:
# it is the thing that keeps the robot on the row at all.
_EMIT_MIN_INTERVAL_S = 0.25
# How far out of date the box may be before the creep distance stops trusting it. The box now
# carries the CAPTURE TIME of its frame, so this is checked against measured age rather than
# against when inference happened to start.
_EMIT_MAX_STALE_M = 0.02
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
# CREEP SPEED IS SET BY THE CAMERA, NOT BY THE MOTORS.
# The camera delivers ~1.4 fps (measured 2026-08-18 with curl straight to it, robot app
# stopped: 11 frames in 8s), so a new measurement arrives every ~710ms. At PWM 77 that is
# 16cm of ground per control update, against a lookahead of ~62cm — a quarter of the
# lookahead consumed blind between corrections, which no gain can stabilise. It also lets
# the robot step clean over the band where an emitter counts as "in reach".
#
# PWM 55 roughly halves that. UNMEASURED: the speed at this duty has not been checked, and
# the two numbers below depend on it —
#   1. _DRIP_SPEED_MPS (distance tracking) was observed at PWM 77 and is now WRONG
#   2. low duty risks stiction; if the robot stalls or crawls unevenly on soil, raise it
# Measure both with:  field_test.py fwd 55 10 0.83   -> metres/10 = m/s
_BASE_PWM = 55
_CAL_PWM  = 77            # the duty _DRIP_SPEED_MPS was observed at
# Threshold on the model's RAW value (see emitter_ml: the old 0.6 scaling plus a bogus moisture
# bonus made 0.55 mean "ml >= 0.25").
#
# 0.90 -> 0.80 on 2026-08-18, for two reasons.
#
# FIRST, 0.90 WAS SET ON MISREAD EVIDENCE. It was chosen to reject three "false positives on
# plain tube" scoring 0.57 / 0.77 / 0.87 — and at least one of those frames CONTAINED REAL
# EMITTER HOLES. They are 4-6px features, invisible at full-frame zoom, and the frame was judged
# without cropping in. So the gate was pushed above a detection that was actually correct.
#
# SECOND, the retrained model measures differently. On 298 held-out frames from the dense pass,
# detections inside the reach band (y >= _EMIT_MIN_Y_FRAC) have a MEDIAN of 0.843 — so 0.90 sits
# above the median of the readings the plant trigger acts on:
#     gate 0.90 -> 19/298 frames qualify  (~3 qualifying frames per emitter over a full pass)
#     gate 0.80 -> 40/298                 (~8 per emitter)
# Three frames per emitter is enough for the edge-triggered plant to fire once, but there is
# almost no margin, and a missed emitter is a skipped seed while a false positive is one wasted
# seed. The costs are not symmetric.
#
# 0.80 -> 0.60 on 2026-08-19, and this time against A FIELD RUN rather than a distribution. The
# note that used to sit here predicted the failure exactly: "too high shows up as the robot walking
# past emitters". It did. A 5m lateral with emitters every 0.40m — about 12 — stopped at TWO.
#
# The per-window emitter tally is what made it legible, and it is worth reading as the model's
# confidence histogram on THIS camera:
#
#     window   frames with a box   passed 0.80   best conf
#      1/1                              1        0.94  y=204   -> stopped
#      7/93                             0        0.78  y=168
#     14/95                             0        0.54  y=78
#      1/4                              1        0.92  y=222   -> stopped
#      0/92     model returned NOTHING on 92 consecutive frames
#     13/95                             0        0.67  y=210
#
# TWO THINGS FOLLOW. The distribution is BIMODAL — 0.92-0.94 or 0.54-0.78, with nothing between
# 0.80 and 0.92 — so 0.80 sat exactly in the empty band and admitted only the top mode. And the
# 92-frame silent window is the control: on plain tube between emitters the model reports nothing
# at all, so the mid-confidence boxes are not noise. It is under-confident, not wrong.
#
# WHY UNDER-CONFIDENT: the model was trained on Logitech C310 frames. The robot now has a
# QHM-999RL — different lens, different field of view (0.43m of ground against the C310's 0.22m),
# different colour rendering. The held-out median of in-reach detections was 0.843 on C310 frames;
# the best-per-window figures above are 0.54-0.78. That is a systematic downward shift, which is
# what a domain change looks like.
#
# So 0.60 is a STOPGAP with a real fix behind it: retrain on frames from this camera. It is the
# right stopgap because the costs are asymmetric — a missed emitter is a skipped seed, a false
# positive is one wasted seed — and because in DRY mode a false stop costs nothing at all.
#
# VALIDATE IT THE SAME WAY: every stop saves an emitN_latM frame. Look at them. Stops on bare tube
# mean 0.60 is too low; a count near 12 on a 5m lateral with all frames on real emitters means it
# is right. Do not raise it again on a distribution.
_EMIT_CONF, _EMIT_COOLDOWN = 0.60, 3.0
# WHERE in frame the emitter must be before we stop for it. The camera looks down AND
# ahead, so an emitter first appears near the TOP of the frame — a metre or more away.
# Confidence alone as the trigger meant the robot stopped the instant each emitter
# entered view, at a different distance every time, and would have dropped seed nowhere
# near the emitter. Lower in frame = nearer, so hold off until the detection's centre is
# below this fraction of the frame height.
_EMIT_MIN_Y_FRAC = 0.55


# WHERE THE PUNCH IS, RELATIVE TO WHAT THE CAMERA CAN SEE:
#     punch tip -> front wheel centre    16 cm   (seeder is centre-mounted; tip sits 3cm
#                                                 behind the pivot axis)
#     front wheel -> bottom of frame     23 cm   (operator measured 24 cm on the new camera)
#     bottom of frame -> top of frame    43 cm   <- CORRECTED 2026-08-19
# So the visible strip is 39-82 cm AHEAD OF THE PUNCH. An emitter under the tip is invisible:
# it left the frame 39 cm ago.
#
# THE FAR EDGE WAS 0.61 AND THAT WAS A C310 NUMBER. Its comment recorded "bottom of frame -> top
# of frame 22 cm", the old camera's strip; the QHM-999RL sees 43 cm, so the far edge is
# 0.39 + 0.43 = 0.82. The error only mattered once the retrained model started detecting away from
# the very bottom, where the two agree — at which point it under-estimated the creep by 7 cm at
# y=162, 10 cm at mid-frame and 18 cm at y=30, all of which the model now reaches.
_PUNCH_TO_FRAME_NEAR_M = 0.39
_PUNCH_TO_FRAME_FAR_M  = 0.82


def _emitter_ground_m(emit, h):
    """Metres from the PUNCH TIP to a detected emitter, or None.

    Maps the detection's row in the image to a ground distance by linear interpolation
    between the two measured frame edges. Ignores perspective, which over a 22cm strip from
    an 18cm-high camera costs about 3cm at mid-frame (~13% of the span) — acceptable for
    dropping a seed beside an emitter, and it is why the trigger prefers detections near the
    BOTTOM of the frame, where both the interpolation error and the creep distance are least.
    To do better, calibrate y-to-distance with an object at two known distances.
    """
    pos = emit.get("position")
    if not pos or not h:
        return None
    frac = max(0.0, min(1.0, pos[1] / float(h)))     # 0 = top of frame, 1 = bottom
    return _PUNCH_TO_FRAME_FAR_M - frac * (_PUNCH_TO_FRAME_FAR_M - _PUNCH_TO_FRAME_NEAR_M)


# WHAT THE EMITTER MODEL ACTUALLY REPORTED, per status window. Added 2026-08-19 after a run did
# 125 inferences down a 5m lateral and stopped at ZERO emitters — with no way to tell which of
# three things had happened:
#
#     the model returned nothing at all              -> retrain, or the wrong model is bound
#     it returned boxes below _EMIT_CONF (0.80)      -> lower the gate, or retrain
#     it returned confident boxes too HIGH in frame  -> _EMIT_MIN_Y_FRAC / geometry, not the model
#
# Three completely different fixes, and the log could not separate them. Two days of emitter
# changes were made without this, which is exactly why they kept missing. detect_emitter_ml asks
# the model with conf_min=0.3, so sub-threshold boxes DO come back and can be counted — the
# information existed and was simply thrown away every frame.
_emit_tally = {"frames": 0, "detected": 0, "conf_ok": 0, "reach_ok": 0,
               "best_conf": 0.0, "best_y": None, "best_frame": None}


def _emit_tally_reset():
    _emit_tally.update(frames=0, detected=0, conf_ok=0, reach_ok=0,
                       best_conf=0.0, best_y=None, best_frame=None)


def _emit_tally_add(emit, in_view, detected, h):
    """Record one frame's emitter observation against each gate in turn."""
    _emit_tally["frames"] += 1
    if not emit.get("detected"):
        return
    _emit_tally["detected"] += 1
    conf = float(emit.get("confidence") or 0.0)
    pos = emit.get("position")
    y = pos[1] if pos else None
    if in_view:
        _emit_tally["conf_ok"] += 1
    if detected:
        _emit_tally["reach_ok"] += 1
    if conf > _emit_tally["best_conf"]:
        _emit_tally.update(best_conf=conf, best_y=y)


def _emit_tally_line(h):
    """One line that says which gate is losing the emitters, or that the model found none."""
    t = _emit_tally
    if not t["frames"]:
        return None
    if not t["detected"]:
        return ("  emitters: model returned NOTHING on %d frames (asked at conf>=0.3). "
                "Either no emitter was in view, or the model cannot see them — check a saved "
                "frame before retraining." % t["frames"])
    reach_px = h * _EMIT_MIN_Y_FRAC if h else 0
    return ("  emitters: %d/%d frames had a box | %d passed conf>=%.2f | %d ALSO low enough "
            "(y>=%.0f) | best conf %.2f at y=%s%s"
            % (t["detected"], t["frames"], t["conf_ok"], _EMIT_CONF, t["reach_ok"], reach_px,
               t["best_conf"], t["best_y"],
               "  <- boxes are too HIGH in frame, not too weak"
               if t["conf_ok"] and not t["reach_ok"] else
               "  <- boxes are below the confidence gate"
               if t["detected"] and not t["conf_ok"] else ""))


def _emit_queue(targets, travelled, gap, dedupe_m=None):
    """Fold one emitter observation into the pending-target list. Pure, so it is testable.

    `gap` is metres from the punch tip to the emitter right now (from _emitter_ground_m). The target
    is therefore `travelled + gap`: the odometer reading at which that emitter will be under the
    punch.

    THE SELF-DEDUPLICATION THAT MAKES THE COUNT RIGHT. As the robot advances, `travelled` grows by
    exactly as much as `gap` shrinks, so one physical emitter produces the SAME target on every frame
    it is seen in, whatever row it occupies. Observations agreeing within `dedupe_m` are therefore
    one emitter, and the estimate is REFINED toward the newest observation — the emitter is nearer
    then, so both the y-to-distance interpolation error and the remaining travel are smaller.

    This replaced a visibility-edge debounce that could not work: with the camera's 0.43 m view wider
    than the 0.40 m emitter spacing there is nearly always an emitter in frame, so "wait until none
    is visible" never came true. It is also why a capped creep was not enough — +/-5 cm placement
    needs the robot to cover the full 0.39 m punch offset, and it can only do that without driving
    past the next emitter by keeping several targets in flight.

    Returns a new sorted list.
    """
    dedupe_m = _EMIT_DEDUPE_M if dedupe_m is None else dedupe_m
    t = travelled + gap
    out = list(targets)
    for i, existing in enumerate(out):
        if abs(existing - t) <= dedupe_m:
            # same emitter, seen again and now nearer: trust the newer estimate more
            out[i] = 0.35 * existing + 0.65 * t
            return sorted(out)
    out.append(t)
    return sorted(out)


def _emit_due(targets, travelled, last_plant_at):
    """The head of the queue, if the robot has reached it. Pure.

    Returns (target, rest) or (None, targets). The _min_replant() floor is a safety net against two
    punches on top of each other; the dedupe in _emit_queue is what actually separates emitters.
    """
    if not targets:
        return None, targets
    head = targets[0]
    if travelled + 1e-9 < head:
        return None, targets
    if travelled - last_plant_at < _min_replant():
        return None, targets
    return head, targets[1:]


def _emitter_in_reach(emit, h):
    """True when the emitter is low enough in frame to act on.

    No longer means "under the seeder" — it cannot, since the seeder is 39cm behind the
    nearest visible ground. It means "near enough that the creep is short and the distance
    estimate is good", after which _creep() brings it under the tip.
    """
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
# GAIN AND FLOOR TOGETHER DECIDE WHETHER THIS IS PROPORTIONAL AT ALL.
# At gain 6 the old command was 6 x err, and real frames give |err| under about 1.8 — so
# the result was below the 11 PWM floor essentially always, and every correction came out
# at exactly +/-11 whatever the error. That is a BANG-BANG controller, which is its own
# source of weaving regardless of how good the measurement is.
# Gain 14 with a floor of 9 puts the proportional region where the errors actually live:
# below err 0.65 the floor still guarantees the robot moves against stiction, and above it
# the command tracks the error. UNVERIFIED IN THE FIELD — these are chosen so a proportional
# band exists, not calibrated.
# Gain and floor are expressed AT _CAL_PWM and scaled to the duty actually in use — a
# differential of 20 PWM on a base of 77 is a gentle correction; the same 20 on a base of
# 55 is a much harder turn, and the same steering command would suddenly oversteer.
_STEER_SCALE    = _BASE_PWM / float(_CAL_PWM)
_STEER_GAIN     = 14 * _STEER_SCALE
_STEER_MIN_DIFF = max(6, int(round(9 * _STEER_SCALE)))
_STEER_DEADBAND = 0.25  # ignore correction noise below this — do not chase pixels
# How much of the error comes from the LOOKAHEAD point rather than the tube at the wheels.
# 0.0 waits until the tube reaches the wheels before correcting; 1.0 ignores lateral offset
# entirely and tracks parallel to the row without ever closing on it.
#
# Retuned 2026-08-18 with the USB camera. Two things changed and they pull the same way:
#   * The near point is genuinely NEAR now — 41cm from the pivot axis, so cross-track error
#     is worth acting on rather than being a stale figure.
#   * The near-to-far baseline shrank from 34cm to 22cm, so `far - near` (the heading signal)
#     is measured over less ground and is correspondingly noisier. Weighting it heavily
#     amplifies that noise.
#   * And at 30fps, frame rate substitutes for lookahead: a correction that is slightly late
#     is re-decided 30 times a second rather than once per 12cm of travel.
# 0.5 splits it evenly. UNVALIDATED IN THE FIELD — start here, and if it tracks parallel but
# offset, lower it; if it hunts, raise it.
_STEER_W_FAR = 0.5
# (The tilt-as-derivative term is gone. far/near carry the same, verified sign convention,
#  and far minus near is the tilt — so the same information arrives without the sign risk
#  that made the earlier angle_deg term make steering worse.)


# ── Temporal gate on the tube reading ────────────────────────────────────────
# A per-frame detector, however good, cannot rule out a reading that is physically
# impossible. The 2026-08-17 run logged the tube at x = 233, 290, 158, 61, 256, 168,
# 14 within about two seconds — the robot creeps at ~0.2 m/s and cannot cross the frame
# in 100ms, so most of those were false whatever the detector said. This keeps the last
# accepted column and rejects anything that jumps further than the robot could have
# moved; after enough consecutive rejections it gives up and re-acquires, so a genuine
# re-acquisition after losing the tube still works.
#
# THESE GATES ARE DERIVED, NOT WRITTEN DOWN. See _gates().
#
# The bug this fixes is not a wrong number, it is a wrong UNIT. Every gate here used to be
# a count of seconds or pixels chosen when the ESP32-CAM delivered a frame every ~710ms.
# The USB camera delivers one every ~33ms, so each of them silently changed meaning by 21x
# the day the camera was swapped, and nothing failed loudly:
#
#   _TRACK_MAX_JUMP_PX = 70   sized for 12-24cm of travel between frames. At 0.57cm it
#                             rejects nothing at all — the backstop against the tube column
#                             teleporting (233 -> 290 -> 158 -> 61) had quietly gone inert.
#   _TRAVERSE_MIN_SIGHTS = 4  over 2.5s. At 1.4fps that is ~every frame in the window and a
#                             strong test; at 30fps it is 4 sightings in 75 frames, a 5% hit
#                             rate — and that latch decides the robot turns onto a row.
#   _TUBE_GRACE_S = 0.4       12 frames. At a 27% miss rate the chance of 12 consecutive
#                             misses is ~1e-6, so the "tube genuinely lost -> stop" fail-safe
#                             could never fire.
#
# So the gates are now expressed in the units the ROBOT cares about — metres of ground and
# frames of evidence — and converted to seconds/pixels at run time from the camera's MEASURED
# interval. A future camera change re-derives them instead of re-breaking them.
#
# RE-ACQUISITION. The jump gate is necessary — it fires on 7% of a real pass — but it needs an
# escape, or after the robot genuinely moves the gate would reject the true new position for
# ever. Two hatches: enough consecutive rejects that AGREE, or an anchor too old to mean
# anything.
#
# THE REJECTS MUST AGREE, and that is the fix. The old rule counted six consecutive rejects and
# then believed WHICHEVER FRAME HAPPENED TO BE SIXTH. Measured over 2086 consecutive frames: of
# 9 relock events, 6 were onto a consistent position (spread 1-22px) and 3 were onto pure
# scatter — spreads of 185px, 79px and 54px. One relock in three was jumping the anchor onto
# noise, which then poisons the gate for every following frame.
#
# So a relock now requires the pending rejects to lie within the jump gate OF EACH OTHER, and it
# anchors on their MEDIAN rather than the last one. Six readings agreeing on a new position is
# real evidence; six scattered readings are evidence of nothing.
#
# (The frame-budget audit predicted trouble here for a different reason — that this hatch and
# the stale-anchor one had swapped which fires first, 4.3s/2-frames at 1.4fps becoming
# 0.23s/39-frames at 26fps. That is true and it is why relock now does the work, but the swap
# on its own was harmless; the missing agreement check was the actual defect.)
_TRACK_RELOCK_N    = 6      # consecutive AGREEING rejects before believing the new position
# How far the robot may travel before the anchor is meaningless regardless. Expressed in metres
# so it survives a speed or frame-rate change; 0.25m is ~1.5s at 0.170 m/s, which is what the
# old flat _TRACK_STALE_S = 1.5 amounted to at this speed.
_TRACK_ANCHOR_MAX_M = 0.25

# Vertical image scale. RE-MEASURED 2026-08-19 FOR THE QHM-999RL webcam: 729.2 px/m, by pushing
# the robot a tape-measured 1.000m and reading the real FlowOdometer (scripts/calib_odometer.py) —
# it reported 0.6690m over 1199 frames with 0 rejected, and went flat for the last 19 seconds,
# which is what proves the push finished. Full derivation in vision/odometry.py.
#
# The old 1090.0 was a Logitech C310 measurement (16mm tube reading 20.9px at the bottom row,
# 13.0px at the top). That camera is gone and this one has a wider lens, so 1090 was the wrong
# camera's constant and made every camera-measured distance read a third short.
#
# Used here to convert "how far has the crossing band moved" into "how far did we drive", so the
# traverse approach gate (_TRAVERSE_APPROACH_FRAC * span_m * _PX_PER_M_DEPTH) was demanding ~1.5x
# more band movement than the geometry actually produces — i.e. it was harder to satisfy than
# intended, which fits traverse never having latched in the field.
_PX_PER_M_DEPTH = 729.2

# HOW FAR THE TUBE COLUMN MAY MOVE, per second of elapsed time rather than per frame.
# 900 px/s puts the gate at ~30px at 30fps and pins it to the ceiling on a slow camera, so
# both cameras get a real test instead of one getting none.
#
# THE FLOOR IS THE INTERESTING PART. Physical motion at 30fps is only ~7.5px (5.7mm of travel
# at 0.76 mm/px), so it is tempting to gate at 10px. Measured against 45 real detections, the
# detector's OWN position noise is p90 7.5px, p99 18.8px, max 22.0px — the measurement is
# NOISIER THAN THE MOTION. Gating below ~25px would therefore reject good frames on noise, and
# no amount of controller tuning recovers a reading that was thrown away. The ceiling is 90px
# because the tube's x_near spanned only 78..229 across an entire run, so a 90px step is
# already most of the operating range.
_TRACK_JUMP_PX_PER_S = 900.0
_TRACK_JUMP_PX_MIN   = 25
_TRACK_JUMP_PX_MAX   = 90
# HOW LONG A DETECTION GAP TO DRIVE THROUGH before stopping.
# The detector achieves 69% on real captures (measured over 80 frames, 2026-08-18) — the 31%
# it misses are frames where the robot's own shadow corrupts one of the four bands, and five
# different discriminators failed to separate those from genuine non-tube frames. But at 30fps
# that still leaves ~20 good readings a second, and the tube does not move in 33ms.
#
# Stopping on the FIRST missed frame is what made the robot halt after ~10cm: it was not
# losing the tube, it was hitting one bad frame in three. So carry the last good reading for
# a short grace period. At 0.170 m/s, 0.4s is 7cm driven on a slightly stale correction —
# far less bad than lurching to a halt three times a second.
#
# This is a tolerance, not a blindfold: past the grace period the robot still stops, which is
# what protects it when the tube genuinely leaves the view.
# Expressed as GROUND DISTANCE plus a cap in FRAMES, whichever runs out first — the two
# things that actually matter, neither of which changes when the camera does.
#
# 0.035m at 0.170 m/s is 0.21s, which at 30fps is ~6 frames. The 6-frame cap agrees, and is
# what bounds it on a faster camera. Six consecutive misses at the measured 27% miss rate has
# probability 0.27^6 = 4e-4, so the grace absorbs shadow gaps while a genuine loss still stops
# the robot within ~3.5cm. The old 0.4s let it drive 6.8cm on a reading that never expired.
_TUBE_GRACE_M          = 0.035
_TUBE_GRACE_MAX_MISSES = 6
# A held reading used to be replayed IDENTICALLY every frame, which is not "carrying a
# measurement through a gap" — it is a 0.4s open-loop turn at whatever differential the last
# good frame asked for. The held correction now decays to zero across the grace window, so the
# robot straightens as its information goes stale instead of committing harder to it.
_tube_hold = {"t": 0.0, "tube": None, "misses": 0}


def _gates(interval=None):
    """Every frame-rate-dependent gate, derived from the camera's MEASURED frame interval.

    Called with bus.interval(); falls back to a conservative 100ms before two frames have
    arrived. _DRIP_SPEED_MPS is read here rather than at module level on purpose — it is
    defined further down the file, and a module-level reference to it would be exactly the
    import-ordering bug that tests/test_console_imports.py exists to catch.
    """
    iv = interval if (interval and interval > 0) else 0.10
    v = max(0.01, _DRIP_SPEED_MPS)
    return {
        "interval": iv,
        # how far the tube column may jump between readings
        "jump_px": max(_TRACK_JUMP_PX_MIN,
                       min(_TRACK_JUMP_PX_MAX, _TRACK_JUMP_PX_PER_S * iv)),
        # how long to carry a lost reading: a distance budget, but never less than one
        # frame's worth or a slow camera would expire the grace before it could ever help
        "grace_s": max(iv * 1.5, _TUBE_GRACE_M / v),
        "grace_misses": _TUBE_GRACE_MAX_MISSES,
        # how far the emitter box may be out of date before it stops setting the creep
        "emit_stale_m": _EMIT_MAX_STALE_M,
        # how long the tracking anchor stays meaningful, as a distance
        "anchor_max_s": _TRACK_ANCHOR_MAX_M / v,
    }


def _gates_str(g):
    return ("frame %.0fms | tube jump <=%.0fpx | grace %.0fms or %d misses (%.1fcm) | "
            "emit stale <=%.0fcm"
            % (g["interval"] * 1000, g["jump_px"], g["grace_s"] * 1000, g["grace_misses"],
               100 * g["grace_s"] * _DRIP_SPEED_MPS, 100 * g["emit_stale_m"]))
# PLAUSIBLE FOR THIS CAMERA. The detector reports what the profile says; how wide the
# tube can look from this camera at this height is the robot's knowledge, not the
# detector's. Measured across every real following frame: 38, 40, 46, 46, 62, 68, 70,
# 70, 78 px. The false positive that sent the alignment loop chasing bare soil on
# 2026-08-17 was 20px. The gap is wide enough to gate on and it costs nothing.
# GATE ON width_fwhm, NOT width. `width` changes meaning depending on whether pair-centring
# succeeded AND it is computed from the WHOLE-FRAME profile, which smears when the tube is
# diagonal — so it collapsed exactly when the robot was off-heading and most needed to steer.
# Measured over 2086 consecutive frames: `width` is bimodal (p10 14, median 44) while
# `width_fwhm` is unimodal with median 14, which agrees with the independent measurement of the
# 16mm tube as a ruler (13-21px across). The old `_TUBE_MIN_W_PX = 30` was therefore rejecting
# 321 of 1010 good detections — 32% — including 4.2-sigma reads of a plainly visible tube.
#
# 5 is a floor against degenerate 1-2px "features", not a real discriminator. It costs 4% recall
# against no gate at all (87% vs 91%) and keeps a sanity check. The actual protection against
# false positives is three-layered and none of it is this number: strength >= 2.5, three-of-four
# band agreement on a fitted line, and _track_tube's temporal jump gate.
#
# STILL A C310 NUMBER, and the only one left (2026-08-19). Everything above — the 13-21px ruler,
# the median of 14 — was measured on the old camera. The QHM-999RL has a WIDER lens: the depth
# scale came out 729 px/m against the C310's 1090, so the same 16mm tube now projects to roughly
# two thirds of its old width, i.e. about 9px rather than 14. A 5px floor still clears that, so
# this is not urgent — but it is 5 against 9 instead of 5 against 14, and the margin has halved.
#
# MEASURE IT before trusting a tube-following run: put the drip tube in frame, save one frame, and
# read `width_fwhm` out of detect_tube. If it comes back near 9 this constant is fine as-is; if it
# comes back at 5-6 the floor is now cutting into real detections and must drop. This cannot be
# derived from the depth scale — lateral and depth scales differ under an oblique view — so it
# needs the tube, which is why it is recorded here rather than guessed at now.
_TUBE_MIN_FWHM_PX = 5
_TUBE_MIN_SIGMA  = 2.5
_track = {"x": None, "rejects": 0, "last_ok": 0.0, "pending": []}
# Which gate is throwing frames away, counted over the log window. THE POINT IS THE RATIO:
# a run that halts constantly looks identical in the log whether the tube is genuinely absent
# or a threshold is miscalibrated, and on 2026-08-18 it was the threshold — for a whole session.
_reject_tally = {}


def _tube_reject(tube):
    """Why this reading is not usable, or None if it is. NAMES THE GATE THAT REJECTED IT.

    Returning a bare boolean is what let a miscalibrated width gate discard 32% of good
    detections through an entire field session: the log could only say "tube lost", and the
    reason had to be reconstructed by comparing numbers printed beside it against a constant
    the reader would have to go and look up. The caller aggregates these strings into a
    histogram, so "94% of rejections were `width`" shows up in one run instead of three.
    """
    if not tube["found"]:
        return tube.get("reject") or "not-found"
    if (tube.get("width_fwhm") or 0) < _TUBE_MIN_FWHM_PX:
        return "fwhm-%s<%d" % (tube.get("width_fwhm"), _TUBE_MIN_FWHM_PX)
    if tube["strength"] < _TUBE_MIN_SIGMA:
        return "sigma-%.1f<%.1f" % (tube["strength"], _TUBE_MIN_SIGMA)
    return None


def _tube_plausible(tube):
    """Is this reading the right SHAPE to be our tube, seen from our camera?"""
    return _tube_reject(tube) is None


def _track_reset():
    _track.update(x=None, rejects=0, last_ok=0.0, pending=[])


def _tube_with_grace(tube, now, gates):
    """Carry the last good reading through a brief detection gap, with DECAY.

    Returns (tube, holding) — `holding` is True when this is a remembered reading rather
    than a fresh one, so the caller can log it and decide how much to trust it.

    Two bounds, whichever runs out first: a ground-distance budget and a count of
    consecutive missed frames. The frame count is what protects a fast camera (where the
    distance budget is many frames) and the distance is what protects a slow one.

    The correction DECAYS linearly to zero across the window. Replaying the last command
    unchanged, as this used to, means the robot keeps turning at full commanded rate on
    information it no longer has — the longer it is blind the harder it commits. Decaying
    means it straightens as the reading ages, so the worst case is driving straight rather
    than driving in a circle.
    """
    if tube["found"]:
        _tube_hold.update(t=now, tube=tube, misses=0)
        return tube, False
    held = _tube_hold["tube"]
    if held is None:
        return tube, False                    # never seen the tube: do not drive on nothing
    age = now - _tube_hold["t"]
    _tube_hold["misses"] += 1
    if age > gates["grace_s"] or _tube_hold["misses"] > gates["grace_misses"]:
        return tube, False
    k = max(0.0, 1.0 - age / max(1e-6, gates["grace_s"]))
    out = dict(held)
    out["correction"] = round((held.get("correction") or 0.0) * k, 3)
    if held.get("far_correction") is not None:
        out["far_correction"] = round(held["far_correction"] * k, 3)
    out["held_for"] = round(age, 3)
    out["held_misses"] = _tube_hold["misses"]
    out["held_decay"] = round(k, 2)
    return out, True


def _tube_grace_reset():
    _tube_hold.update(t=0.0, tube=None, misses=0)


def _track_tube(tube, w, now, max_jump_px, anchor_max_s=1.5, hinted=False):
    """Accept, reject or re-acquire this frame's tube reading.

    `max_jump_px` comes from _gates() rather than a constant, because how far the tube can
    plausibly move depends entirely on how long ago the last reading was taken.

    `hinted` only affects the LOG. A reject means two opposite things: cold, the detector had
    nothing to go on and losing the tube among stronger features is expected; hinted, it was
    told within 45px of where the tube actually was and STILL could not find it, which is a real
    detector failure. Reading them off one undifferentiated histogram, as we did all day, makes
    a run of expected cold misses look identical to the detector being broken.
    """
    why = _tube_reject(tube)
    if why is not None:
        _track["rejects"] = 0            # nothing to disagree with; do not count it
        _reject_tally[why] = _reject_tally.get(why, 0) + 1
        if hinted:
            _reject_tally["(of those, hinted)"] = \
                _reject_tally.get("(of those, hinted)", 0) + 1
        out = dict(tube)
        out["reject"] = why
        if tube["found"]:                # found, but not tube-shaped: say so
            out.update(found=False, correction=0.0, far_correction=0.0, implausible=True)
        return out
    x = tube["tube_x"]
    prev = _track["x"]
    stale = (now - _track["last_ok"]) > anchor_max_s
    if prev is None or stale or abs(x - prev) <= max_jump_px:
        _track.update(x=x, rejects=0, last_ok=now, pending=[])
        return tube

    # Rejected. Remember WHERE it wanted to go, because a run of rejects that agree with each
    # other is a real re-acquisition and a run that scatters is noise — and the old code could
    # not tell the difference.
    pend = _track["pending"]
    pend.append(x)
    if len(pend) > _TRACK_RELOCK_N:
        del pend[0]
    _track["rejects"] += 1
    if len(pend) >= _TRACK_RELOCK_N and (max(pend) - min(pend)) <= max_jump_px:
        anchor = sorted(pend)[len(pend) // 2]        # median, not the newest
        _track.update(x=anchor, rejects=0, last_ok=now, pending=[])
        out = dict(tube)
        out["relocked_to"] = anchor
        return out

    out = dict(tube)
    out.update(found=False, correction=0.0, far_correction=0.0, rejected_x=x,
               reject_run=_track["rejects"],
               pending_spread=round(max(pend) - min(pend), 1) if len(pend) > 1 else 0.0)
    return out


def _steer(tube):
    """Differential PWM from a LOOKAHEAD-weighted error.

    The robot used to steer on the tube's position averaged over the whole frame, and it
    would not correct until the tube had reached the wheels — then it corrected hard,
    overshot, and zig-zagged. That is a structural fault, not a gain that needed tuning:
    with the robot still over the tube at the wheels but pointing off, the displacement is
    ALL at the top of the frame, so the average halves it and the half it keeps describes
    where the robot IS rather than where it is GOING.

    So the error is a blend of two points the detector now reports separately: the tube at
    the robot (cross-track) and the tube at the top of the frame (lookahead). Weighted
    toward the lookahead, the robot starts correcting while the error is still small, which
    is what removes the oscillation; the cross-track term is what stops it settling
    parallel to the row but offset from it. This is the standard geometry for row guidance
    — Pure Pursuit / Stanley — and lookahead is the parameter that matters.

    Both terms use `correction` units and the SAME sign convention ("+ = tube is right ->
    steer right"), deliberately: an earlier version fed the tube's tilt angle in as a
    derivative term and made steering worse, because the angle's sign was never verified
    against the robot. far minus near IS that tilt, expressed in units whose sign is
    already pinned by the detector's contract.

    NOTE the lookahead is only as good as the camera geometry, which is unmeasured — the
    weights below are a starting point, not a calibration. Too little lookahead hunts
    (where we started); too much cuts corners and ignores real offset.
    """
    near = tube.get("correction") or 0.0
    far = tube.get("far_correction")
    if far is None:                       # detector without a lookahead point
        far = near
    err = _STEER_W_FAR * far + (1.0 - _STEER_W_FAR) * near
    if abs(err) < _STEER_DEADBAND:
        err = 0.0
    d = _STEER_GAIN * err
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
_DRIP_SPEED_MPS = CAL.get("creep") or (0.225 * _BASE_PWM / float(_CAL_PWM))
if CAL.get("creep") and CAL.get("creep_pwm") != _BASE_PWM:
    print("WARNING: creep speed %.3f m/s was measured at PWM %s but _BASE_PWM is %d — "
          "re-run `field_test.py fwd %d 10 0.83` and update CAL[\"creep\"], or every "
          "distance this run reports will be wrong."
          % (CAL["creep"], CAL.get("creep_pwm"), _BASE_PWM, _BASE_PWM), flush=True)

# THE MOISTURE PROBE IS NOT FITTED, so it must not vote on emitter detection.
# uno-q-wiring.md: "The seeder uses D10/A3/D11/D13; the probe uses A4/A5/D2" — the two are
# mutually exclusive attachments, and A4/A5 now carry the gyro. So during a seeding run A0/A1
# are FLOATING, and they read 1855/2099 out of 16383, which is below the 9000 "wet" threshold.
# detect_emitter_ml adds +0.4 confidence when wet, so every frame got that bonus for free:
#   conf = 0.6 x ml_value + 0.4
# which means an ml_value of only 0.25 clears the 0.55 gate. Reading back from the three
# false positives logged on 2026-08-18 (conf 0.92/0.74/0.86 on frames showing PLAIN TUBE):
#   ml was 0.87 / 0.57 / 0.77  ->  without the bonus: 0.52 / 0.34 / 0.46, all REJECTED.
# So a disconnected sensor was manufacturing the confidence that let plain tube pass.
MOISTURE_PRESENT = False


def _moisture_min():
    if not MOISTURE_PRESENT:
        return None                 # None -> detect_emitter_ml leaves `wet` False
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
    # FALL BACK TO THE LAST KNOWN IP, even expired. The helper timed out once on
    # 2026-08-18 and the loop got the bare name back, which cannot resolve inside this
    # container — so the camera failed to open at all and stayed down until someone
    # pressed Connect. A stale IP is nearly always right (the cam holds its DHCP lease)
    # and is strictly better than a name we know will fail.
    if hit:
        log("  using last known %s -> %s (resolve failed)" % (name, hit[1]))
        return hit[1]
    return name


def _host(hostport):
    """Swap a bare *.local host for its IP, preserving any :port suffix."""
    host, _, port = hostport.partition(":")
    if host.endswith(".local"):
        host = _resolve_mdns(host)
    return host + (":" + port if port else "")


def _stream_url(base):
    """Accept 'ip', 'host.local', 'http://…', a full URL, or a USB camera -> source spec.

    A USB camera is passed through UNCHANGED for FrameBus to route. Without this, '2' went
    through the mDNS/host path and came out as 'http://2:81/stream'.
    """
    b = str(base).strip()
    if not b:
        return ""
    if b.lower() in ("usb", "auto", "webcam") or b.startswith("/dev/video") or b.isdigit():
        return b                              # USB camera: FrameBus._open handles it
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
    """Guarded entry point. WHATEVER happens inside, release the camera and stop the wheels.

    This wrapper exists because of a specific, expensive failure on 2026-08-18: an
    UnboundLocalError on the first frame killed the loop thread, and because `bus.stop()`
    was the last STATEMENT of the loop rather than a guaranteed one, the FrameBus reader
    thread survived and kept streaming the V4L2 device. A V4L2 camera streams to one
    consumer, so every subsequent Connect opened the same device, got no frames, and leaked
    another orphan reader — the camera stayed dead until the container was restarted, and
    the log only ever showed "camera open failed".

    Two guarantees, and they are worth more than the typo that motivated them:
      * THE CAMERA IS ALWAYS RELEASED, so a crash costs one loop, not the device.
      * THE MOTORS ARE ALWAYS STOPPED. The old code left the last setMotors latched when
        the loop exited for ANY reason — a crash, or the operator simply changing camera
        mid-run. Same fail-safe as "no frame = no eyes -> stop", one level further out.

    `held` is a cell rather than a return value because the bus must be reachable from the
    finally even when the body raised before it could return anything.
    """
    held = {}
    try:
        _cam_loop_run(url, held)
    except Exception:                                          # noqa: BLE001
        # PRINT THE TRACEBACK. A bare thread exception goes to stderr and is easy to miss
        # in the container log; routing it through log() puts it in the operator's log too,
        # beside the frames and decisions that led up to it.
        log("camera loop CRASHED — releasing the camera and stopping the motors:\n%s"
            % traceback.format_exc())
    finally:
        # STOP THE WORKER BEFORE THE BUS. It reads bus.latest() in a loop, and tearing the bus
        # out from under it would just log an exception per iteration until it noticed.
        worker = held.get("emit")
        if worker is not None:
            try:
                worker.stop()
            except Exception:                                  # noqa: BLE001
                log("camera loop exit: emitter worker stop raised:\n%s"
                    % traceback.format_exc())
        bus = held.get("bus")
        if bus is not None:
            try:
                bus.stop()
            except Exception:                                  # noqa: BLE001
                log("camera loop exit: bus.stop() raised:\n%s" % traceback.format_exc())
        try:
            _drive_stop("cam-loop-exit")
        except Exception:                                      # noqa: BLE001
            log("camera loop exit: could not stop the motors:\n%s" % traceback.format_exc())


def _cam_loop_run(url, held):
    emit_worker = None          # bound BEFORE anything can read it (see the `holding` bug)
    # MEASURE-ONLY. Integrates the ground the camera says the robot covered, alongside the
    # wall-clock x _DRIP_SPEED_MPS figure the run actually acts on. Nothing reads it yet: the
    # method is validated (response median 0.641 on 2085 real pairs, stationary frames read
    # 0.06px) but it has never been compared against a KNOWN distance at creep duty, and
    # _DRIP_SPEED_MPS is the number that decides where a row ends. Drive a measured distance,
    # compare the two lines in the log, then promote it.
    odo = FlowOdometer()
    travelled = 0.0             # estimated distance along the lateral this run
    last_plant_at = -1e9        # travelled-at-last-plant, so the first emitter is never gated
    emit_targets = []           # conveyor: odometer readings at which an emitter reaches the punch
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
    loop_t0, loop_n = time.time(), 0     # camera-loop rate, logged during a run
    grace_n = 0                          # frames driven on a HELD reading (shadow gaps)
    _susp_n = 0                          # accepted frames demanding a near full-scale steer
    bus_frames0 = 0                      # camera frame count at the last rate log
    last_cross_log = 0.0
    prev_run_state = None       # to detect a FRESH run and reset per-run counters
    last_push = 0.0
    push_interval = 1.0 / _BROWSER_FPS
    # A THREAD OWNS THE CAMERA NOW. This loop blocks for seconds at a time — 2s of plant
    # dwell per emitter, ~1.5s per pivot, ~1.7s creeping onto a lateral — and while it did,
    # nobody read the socket. The camera then could not finish a send, abandoned the frame
    # ("Stream ends prematurely at 5675") and the stream got reopened; the frame after a
    # reopen decodes into a fresh zero-filled buffer, which is what the green patches are.
    # The bus also timestamps frames, so this loop can tell a live view from a 2s-old one —
    # which it previously had no way to do at all.
    try:
        bus = FrameBus(_stream_url(url), reopen_after_s=_CAM_STALE_S, on_log=log)
        held["bus"] = bus       # reachable from the finally BEFORE start() can raise:
        bus.start()             # __init__ opens nothing, start() is what claims the device
        # Inference gets its own thread off the same bus. `detect` closes over the ML/classical
        # fallback so the worker itself stays testable with a stub detector.
        #
        # CAREFUL IF THE MOISTURE PROBE IS EVER REFITTED. _moisture_min() is handed to a second
        # thread here. It is safe today only because MOISTURE_PRESENT is False, so it returns
        # None without touching the Bridge — and the probe cannot be refitted anyway while the
        # gyro holds A4/A5. Setting MOISTURE_PRESENT = True would start issuing Bridge RPCs
        # from this thread concurrently with the control loop's, which the RouterBridge link has
        # never been exercised for. Read the probe on the control thread and pass the value in.
        def _detect_emit(frame, moisture=None):
            r = detect_emitter_ml(frame, moisture=moisture) if ml_available() else None
            if r is None or not r.get("ml_ready"):
                r = detect_emitter(frame, moisture=moisture)
            return r
        emit_worker = EmitterWorker(bus, _detect_emit, moisture=_moisture_min,
                                    min_interval=_EMIT_MIN_INTERVAL_S, on_log=log)
        held["emit"] = emit_worker
        emit_worker.start()
    except Exception as e:
        print("camera open failed: %s" % e, flush=True)
        return
    print("camera loop (frame bus): %s  [emitter: %s]"
          % (_stream_url(url), "ML/Edge-Impulse" if ml_available() else "classical CV"),
          flush=True)
    fail_since = None
    last_frame_t = 0.0
    while _cam["url"] == url and _VISION_OK:
        t_frame, frame = bus.latest()
        fresh = frame is not None and bus.age() <= bus.stale_after()
        if frame is not None and t_frame == last_frame_t and fresh:
            time.sleep(0.005)         # nothing new yet; do not re-process the same frame
            continue
        if not fresh:
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
            # The bus reopens the stream by itself; this is the escalation when reopening
            # is not working, and it has to stay because a thread that is ALIVE but useless
            # made Connect a no-op (2026-08-15, seven clicks, nothing).
            fail_since = fail_since or time.time()
            if time.time() - fail_since > _CAM_STALE_S * 2:
                if bus.open_failures > _CAM_REOPEN_TRIES or not bus.alive():
                    log("camera dead (%d reopens, %d open failures) — dropping the loop "
                        "so Connect can start a fresh one"
                        % (bus.reopens, bus.open_failures))
                    _cam["url"] = ""          # UI shows disconnected; next Connect is clean
                    break
                fail_since = time.time()
            time.sleep(0.1)
            continue
        fail_since = None
        last_frame_t = t_frame
        _cam["last_frame"] = t_frame
        now = time.time()
        h, w = frame.shape[:2]
        # DETECT FIRST, THEN COUNT WHAT THE DETECTION DID. `holding` comes out of
        # _tube_with_grace and is read by the rate log immediately below; with the log
        # ordered first it was read before it was ever assigned, so the loop died with
        # UnboundLocalError on its FIRST fresh frame and took the camera down with it
        # (2026-08-18 — see the guard on _cam_loop for the other half of that failure).
        # DERIVE THE GATES FROM WHAT THE CAMERA IS ACTUALLY DOING, every frame. Cheap
        # arithmetic, and it means a camera that slows down (or is swapped) re-tunes the
        # plausibility limits instead of silently invalidating them.
        g = _gates(bus.interval())
        odo.update(frame)          # ~1-2ms; measure-only, see the note at the top
        # PASS THE LAST ACCEPTED COLUMN IN. Without this each band takes the strongest peak
        # anywhere in the frame and lands on shadow edges, straw or sunlit boundaries — which
        # on 2026-08-18 froze the robot for 36s at _MIN_BANDS=4 and steered it off the row at 3.
        # _track["x"] is None until the first accepted reading, which is exactly the cold-start
        # case where there is nothing to hint with.
        # EXPIRE THE HINT with the anchor. detect_tube does NOT fall back to a full search while
        # hinted (measured: the fallback returned a band 80px off the tube where reporting nothing
        # was correct), so a hint that never cleared would block re-acquisition for ever once it
        # drifted off the tube. This timeout — the same one the jump gate uses — is what makes a
        # genuine loss recoverable.
        _hint = _track["x"] if (now - _track["last_ok"]) <= g["anchor_max_s"] else None
        tube = _track_tube(detect_tube(frame, hint_x=_hint), w, now=now,
                           max_jump_px=g["jump_px"], anchor_max_s=g["anchor_max_s"],
                           hinted=_hint is not None)
        tube, holding = _tube_with_grace(tube, now, g)
        # Loop rate, logged while a run is active. The whole point of decoupling the
        # emitter model was to raise this, so it should be visible rather than assumed.
        loop_n += 1
        if holding:
            grace_n += 1
        if _run["state"] == "running" and now - loop_t0 >= 5.0:
            # Report the CAMERA's rate beside the loop's. They answer different questions:
            # if the camera is delivering 15fps and the loop runs at 1, the control loop is
            # the bottleneck; if the camera itself is at 1fps, no amount of loop tuning
            # helps and the problem is the stream. `dropped` is corrupt frames refused.
            cam_n = bus.frames - bus_frames0
            iv = bus.interval()
            log("  loop %.1f fps | camera %.1f fps (frame every %s) | tube held %d%% of "
                "frames (shadow gaps) | %d dropped, %d reopens"
                % (loop_n / (now - loop_t0), cam_n / (now - loop_t0),
                   "%.0fms" % (iv * 1000) if iv else "?",
                   int(100.0 * grace_n / max(1, loop_n)), bus.dropped, bus.reopens))
            # THE DERIVED GATES, beside the rate they were derived from. The whole class of
            # bug this replaces was invisible precisely because nothing ever printed the
            # gate next to the frame interval that decides what it means.
            log("  gates: %s" % _gates_str(g))
            # WHICH GATE IS DISCARDING FRAMES, as a ratio. "tube held 60%" tells you the robot
            # is struggling; this tells you WHY, and it is the line that would have caught the
            # width-gate bug on its first run instead of its fifth.
            # THE CALIBRATION LINE. `travelled` is wall-clock x an assumed speed and gates
            # end-of-row; `camera` is what the ground actually did. A run told to cover 5m
            # covered about 7m and this is the number that would have said so at the time.
            od = odo.stats()
            log("  distance: travelled %.2fm (assumed %.3f m/s) | camera %.2fm "
                "(%d frames, %d rejected) -> ratio %s"
                % (travelled, _DRIP_SPEED_MPS, od["distance_m"], od["updates"],
                   od["rejected"],
                   "%.2fx" % (travelled / od["distance_m"]) if od["distance_m"] > 0.05
                   else "n/a"))
            if _susp_n:
                # ~9% is NORMAL (measured, see the screen above). Read this as "how many frames
                # are worth eyeballing", not as a fault count.
                log("  suspect accepts: %d frame(s) jumped >=60%% of the gate "
                    "(first 12 saved as suspect-*.jpg; ~9%% is normal)" % _susp_n)
            _et = _emit_tally_line(h)
            if _et:
                log(_et)
            _emit_tally_reset()
            if _reject_tally:
                # The hinted count is a SUBSET of the causes above, not a cause itself, so it is
                # kept out of both the total and the top-4 — otherwise it would double-count and
                # could crowd out a real cause in the ranking.
                hinted_n = _reject_tally.pop("(of those, hinted)", 0)
                tot = sum(_reject_tally.values())
                top = sorted(_reject_tally.items(), key=lambda kv: -kv[1])[:4]
                log("  rejects: %d in %.0fs — %s | %d while HINTED (%d%%)%s"
                    % (tot, now - loop_t0,
                       ", ".join("%s %d%%" % (k, round(100.0 * v / tot)) for k, v in top),
                       hinted_n, round(100.0 * hinted_n / tot) if tot else 0,
                       " <- detector failing with a good hint" if hinted_n > tot * 0.5 else ""))
                _reject_tally.clear()
            # The worker's own numbers. `latency` is what the control loop USED to pay per
            # expiry and no longer does, so it is worth seeing; `skipped` being large is
            # healthy — it means inference is keeping up with the bus rather than falling
            # behind it.
            if emit_worker is not None:
                st = emit_worker.stats()
                log("  emitter worker: %d inferences, %sms each, %d errors, box age %.0fms"
                    % (st["runs"], st["latency_ms"], st["errors"],
                       1000 * min(9.999, emit_worker.age(now))))
            loop_t0, loop_n, grace_n, bus_frames0 = now, 0, 0, bus.frames
        elif _run["state"] != "running":
            loop_t0, loop_n, grace_n, bus_frames0 = now, 0, 0, bus.frames

        # THE EMITTER MODEL IS NOT ON THIS THREAD AT ALL ANY MORE.
        # It runs in EmitterWorker off the same FrameBus, so this loop never waits 80ms for a
        # model that steering does not use, and the answer arrives stamped with the CAPTURE
        # TIME of the frame it was computed from. That stamp is the point: staleness is now
        # measured rather than inferred from when inference last started, so the creep distance
        # can be trusted or refused on evidence. See vision/emitter_worker.py.
        emit_t, emit = emit_worker.latest() if emit_worker else (0.0, None)
        if emit is None:
            emit = {"detected": False, "position": None, "confidence": 0.0,
                    "visual": False, "wet": False}
        _cam.update(w=w, h=h, tube=tube, emitter=emit)

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
            odo.reset()
            cross_hist = []
            _susp_n = 0             # a second run must not inherit the first run's save budget
            _emit_tally_reset()
            _track_reset()
            _tube_grace_reset()
            last_plant_at, armed, driving_since = -1e9, True, None
            emit_targets = []
        prev_run_state = _run["state"]

        # Tube found/lost EDGE logging. Logging every frame would flood at frame rate,
        # but logging nothing left "the robot moved 5cm and stopped" with no trace at
        # all in the log (2026-08-15) — the stall was invisible. So: log the two
        # transitions only, and only while a follow-run is active, otherwise an idle
        # camera would chatter every time something passes in front of it.
        _following = (_run["state"] == "running"
                      and (_run["mode"] == "scan"
                           or (_run["mode"] == "drip" and phase == "follow")))

        # SUSPICIOUS ACCEPTS — a SCREEN, not a detector, and deliberately not a threshold I
        # reasoned my way to. Everything else logged here records frames we REJECTED: the reject
        # tally, the lost-frame dump, the LOST/REGAINED edges. A false positive is ACCEPTED, so it
        # appears in none of them and produces no found->not-found edge either. It is structurally
        # invisible, which is why the crossing-lateral false positive of 2026-08-18 could only be
        # found offline against saved frames.
        #
        # MEASURED, over 700 real following frames (captures/dense, 408 accepted consecutive
        # pairs), because my first two guesses at a trigger were both wrong:
        #
        #   |correction| is USELESS here. I assumed it ran -1..+1 and called the false positive's
        #   -0.98 "near full-scale". The contract is -5..+5 (vision.py:287), and real following
        #   frames have |corr| p50 1.59, p90 4.41. The false positive is BELOW the median. Same
        #   trap as the gyro spike: magnitude cannot separate signal from noise here either.
        #
        #   The JUMP is better but not clean: p50 2.0px, p90 26.6px, p99 61.7px. The false
        #   positive jumped 31.2px, which is 66% of the 47px gate at 19fps — p92, not an outlier.
        #
        # So ~9% of perfectly normal frames trip this. That is accepted deliberately: the SAVES
        # are capped at 12, which is enough evidence to eyeball afterwards, and the counter keeps
        # reporting past the cap. A screen that catches the event inside a manageable pile of
        # frames beats an alarm tuned so tight it misses it. There is no single-frame test that
        # separates a diagonal crossing lateral from a genuine leaning tube — the distinguishing
        # fact is temporal — so do not expect one here.
        _jump = abs((tube.get("x_near") or 0.0) - _hint) if (
            _hint is not None and tube["found"]) else None
        if _following and _jump is not None and _jump >= 0.6 * g["jump_px"]:
            _susp_n += 1
            if _susp_n <= 12:          # bounded: a bad row must not fill the board's disk
                _shot = _save_named(frame, "suspect")
                log("  SUSPECT ACCEPT [%s] jumped %.0fpx (%.0f%% of the %.0fpx gate) — "
                    "x_near=%.0f x_far=%.0f corr %+.2f angle=%.1f fwhm=%s width=%s bands=%s. "
                    "Either a real bend, or the detector moved to another feature; a crossing "
                    "lateral does this (see tests/frames/negative)."
                    % (_shot, _jump, 100.0 * _jump / g["jump_px"], g["jump_px"],
                       tube.get("x_near") or 0.0, tube.get("x_far") or 0.0,
                       tube.get("correction") or 0.0, tube.get("angle_deg") or 0.0,
                       tube.get("width_fwhm"), tube.get("width"), tube.get("bands_raw")))
            elif _susp_n == 13:
                log("  SUSPECT ACCEPT: 12 frames saved, further ones counted only.")

        if not _following:
            tube_seen = None                  # next run starts clean, logs its first edge
        elif tube["found"] != tube_seen:
            tube_seen = tube["found"]
            if tube_seen:
                log("tube REGAINED (near=%s far=%s px, corr near %+.2f far %+.2f, "
                    "tilt %s) — driving resumes"
                    % (tube.get("x_near"), tube.get("x_far"),
                       tube.get("correction") or 0.0,
                       tube.get("far_correction") or 0.0, tube.get("angle_deg")))
            else:
                # SAVE THE FRAME WE FAILED ON. Until now only SUCCESSES were kept — the
                # emitN_latM frames are written when the robot stops to plant — so a run
                # that lost the tube left no evidence of why, and the shadowed-tube report
                # of 2026-08-17 could not be investigated at all. The failures are the
                # frames worth having: every detector fix today came from looking at real
                # pixels, and twice reasoning without them sent us the wrong way.
                shot = _save_named(frame, "lost")
                # The emitter branch is an `elif` after this one, so NO TUBE means the
                # emitter test never runs at all — during the 2026-08-18 run that made
                # emitter detection structurally unreachable, and the log gave no hint of
                # it. Reporting what the model saw here separates "the model found nothing"
                # from "we never asked".
                # NAME THE GATE. "found=False w=12 s=4.2" required the reader to know that 12
                # is compared against a constant called _TUBE_MIN_W_PX living 900 lines away.
                # "REJECTED BY fwhm-4<5" does not.
                log("  lost-frame saved [%s] — REJECTED BY %s | fwhm=%s width=%s(paired=%s) "
                    "s=%.1f bands=%s | emitter conf %.2f (not evaluated: no tube)%s"
                    % (shot, tube.get("reject") or "?", tube.get("width_fwhm"),
                       tube.get("width"), tube.get("paired"),
                       tube.get("strength") or 0.0, tube.get("bands_raw"),
                       emit.get("confidence") or 0.0,
                       ", jump rejected (x=%s)" % tube.get("rejected_x")
                       if tube.get("rejected_x") is not None else ""))
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
            cross_hist, tr = _traverse_track(cross_hist, traversed, cross)
            approaching = tr["approaching"]
            sights, frac, grew = tr["sights"], tr["frac"], tr["grew"]
            span_m, need_px = tr["span_m"], tr["need_px"]
            arrived = cross["found"] and cross["nearness"] >= _TRAVERSE_NEAR

            # SAY WHAT WE SEE. A field run drove over two real laterals and reported
            # only "no next lateral found" — no way to tell whether the detector saw
            # nothing, or saw them and rejected them. Logged ~1/s so the next run
            # produces numbers to tune against.
            if now - last_cross_log >= 1.0:
                last_cross_log = now
                # SAVE THE FRAME BEHIND EVERY TRAVERSE TICK. Added 2026-08-19 after a traverse
                # failure could not be diagnosed at all: drip mode does not enable dataset capture,
                # so a 1.6m traverse left exactly ONE frame — the decision frame — and the whole
                # sequence that led to it was gone. The numbers said a crossing was "found" at
                # nearness 0.93 from the very first frame and its y never moved (223 -> 218 -> 221
                # -> 219 -> 218 -> 214 over 1.2m of driving), which is a feature fixed relative to
                # the ROBOT rather than the ground; and the one frame we did keep contains no
                # lateral at all. But with one frame there is no way to test a fix.
                #
                # ~1/s matches the log tick, so a 2m traverse costs about a dozen 25KB frames.
                # The name carries the traversed distance, so a frame can be matched to its log line.
                _tshot = _save_named(frame, "trav%d_%03dcm" % (lateral + 2, int(traversed * 100)))
                log("  traverse %.2fm | found=%s y=%s near=%.2f w=%s s=%.1f %s | "
                    "sights %d/%d (%.0f%%, need %.0f%%) | grew %+.0fpx over %.2fm "
                    "(need %.0fpx)%s"
                    % (traversed, cross["found"], cross["tube_y"], cross["nearness"],
                       cross["width"], cross["strength"], cross["polarity"] or "-",
                       # sights and frames are ALREADY COUNTS. _traverse_track returns
                       # {"sights": len(sights), "frames": len(hist)}, and this line wrapped both
                       # in len() again -> TypeError: object of type 'int' has no len(). It killed
                       # the camera loop on the FIRST traverse log line, which is why the robot
                       # stopped dead after every row turn and why traverse has never once latched
                       # in the field. Introduced when _traverse_track was extracted as a pure
                       # function without updating this caller, and invisible because no test and
                       # no run had reached the traverse branch since.
                       sights, tr["frames"], 100 * frac,
                       100 * _TRAVERSE_MIN_SIGHT_FRAC, grew, span_m, need_px,
                       ("" if far_enough else "  (still inside min-traverse)")
                       + (" [%s]" % _tshot if _tshot else "")))

            if far_enough and approaching and arrived:
                _drive_stop("drip")
                turn_right = (lateral % 2 == 0)       # same way as the first turn
                pre = _save_named(frame, "lat%d_found" % (lateral + 2))
                log("next lateral found after %.2fm (nearness %.2f, y=%s, w=%s, "
                    "%.1f sigma %s) — turning on [decision frame %s]"
                    % (traversed, cross["nearness"], cross["tube_y"], cross["width"],
                       cross["strength"], cross["polarity"], pre))
                reached = _arrive_over_lateral(bus)   # get the pivot axis ONTO the tube
                log("  arrived over the lateral (nearness reached %.2f, then %.2fm "
                    "of camera lookahead) — pivoting %s"
                    % (reached, _ARRIVE_EXTRA_M, "right" if turn_right else "left"))
                _track_reset()                        # new row: do not gate on the old x
                _turn_onto_tube(bus, turn_right, "lat%d" % (lateral + 2))
                lateral += 1
                phase = "follow"
                travelled = 0.0
                last_plant_at = -1e9                  # first emitter of a row is never gated
                armed = True
                emit_targets = []                     # a new lateral starts with an empty conveyor
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
            # Debounce on the DETECTION EDGE, not on an assumed spacing: the model is what finds
            # emitters, so "this one again" means "still nearby". _min_replant() is only a floor
            # against double-counting one emitter — it never TRIGGERS a plant, so a missed emitter
            # simply means the next one is found normally.
            #
            # RE-ARM ON LEAVING THE REACH ZONE (`detected`), NOT THE FRAME (`in_view`). This was
            # `if not in_view` and the camera swap broke it:
            #
            #     camera ground coverage : 0.43 m   (measured on the QHM-999RL)
            #     emitter spacing        : 0.40 m   (drip_config emitter_gap)
            #
            # With the view wider than the spacing there is ALWAYS an emitter somewhere in frame,
            # so `in_view` never went False, `armed` never came back after the first plant, and a
            # 5m lateral produced exactly ONE emitter — which is what the 02:26 run did (1 emitter
            # over 5.01m where ~12 were expected). The old C310 saw a 22cm strip, so at 40cm
            # spacing there were long gaps with nothing in view and frame-exit re-arming worked.
            # Nothing about the logic changed; the lens did.
            #
            # THE COMMENT THIS REPLACES warned against exactly this change: "otherwise the same
            # emitter re-arms the moment it drops below the reach line and gets planted twice".
            # That warning predates _min_replant(), which was added afterwards for precisely that
            # failure (the 03:33 run stopped at 2.73m and 2.84m on one emitter) and now covers it:
            # 0.20 m at a 0.40 m spacing is comfortably above the re-entry transient and
            # comfortably below a real gap. Re-entry is also impossible in practice — the robot
            # only ever moves forward, so an emitter that has dropped out of the bottom of the
            # reach zone cannot come back into it.
            _emit_tally_add(emit, in_view, detected, h)

            # THE CONVEYOR. Every confident observation becomes a target distance and joins the
            # queue; the punch fires when the robot reaches each one. `detected` (the reach-zone
            # test) is no longer the trigger — it only decides which observations are trustworthy
            # enough to queue, because a detection near the bottom of frame has the least
            # y-to-distance interpolation error and the shortest remaining travel.
            if detected:
                _gap_now = _emitter_ground_m(emit, h)
                if _gap_now and _gap_now > 0.0:
                    emit_targets = _emit_queue(emit_targets, travelled, _gap_now)
            _due, _rest = _emit_due(emit_targets, travelled, last_plant_at)
            plant_ok = _due is not None

            if not tube["found"]:
                # NO TUBE IN VIEW -> STOP, whatever else is happening this frame.
                # This used to be the last `else` of an if/elif chain, so a frame with
                # an emitter detected-and-armed but inside the re-plant floor matched
                # NO branch: no drive call, no stop call, and the previous setMotors
                # stayed latched. The robot drove on with nothing in view until the
                # operator hit Stop. Stopping is now decided first, not last.
                _drive_stop("drip")
                driving_since = None
            elif plant_ok:
                _drive_stop("drip")
                # CLEAR THE DRIVE CLOCK, or the whole stop gets billed as forward travel.
                #
                # This branch blocks for seconds — the creep to bring the emitter under the punch
                # (~2.5s), the arm dwell, and in dry mode a 2s hold — and `travelled` is integrated
                # as (now - driving_since) * _DRIP_SPEED_MPS on the next frame that drives. With
                # driving_since left stale, that integral spans the entire stop: ~0.84m of phantom
                # travel PER EMITTER, on top of the `travelled += gap` below which already counts
                # the creep properly.
                #
                # The two stop paths around this one both clear it; this one did not, and the cost
                # is not cosmetic because line ~1882 ends the lateral on `travelled >= _plot["l"]`.
                # Measured on the 02:58 run: three emitters, travelled reported 5.37m while the
                # calibrated camera odometer said 3.55m — a 1.82m over-count that ended a 5m
                # lateral at about 3.5m of real ground. That is exactly the "robot stopped at 3m
                # when it should go to 5m" symptom, and the `distance: ... ratio` line drifting
                # 0.92x -> 1.40x -> 1.51x across the run is the same fault seen from the other end.
                driving_since = None
                # THE frame this stop was taken on, kept so a run can be audited after
                # the fact: was it a real emitter, and was the robot actually on top of
                # it? A confidence number in a log line cannot answer either.
                shot = _save_named(frame, "emit%d_lat%d" % (_run["planted"] + 1, lateral + 1))

                # NO CREEP. The conveyor already drove the emitter to the punch: this branch only
                # fires when `travelled` reaches the target that was computed when the emitter was
                # seen, so the distance has been covered WHILE STILL WATCHING rather than blind.
                #
                # What was here before: stop the moment an emitter entered the reach zone, then
                # blind-creep _emitter_ground_m() metres to bring it under the tip. The creeps came
                # out at 0.43-0.51 m against a 0.40 m emitter spacing, so every stop drove past the
                # next emitter with the camera unconsulted — 7 stops over a row holding 13 emitter
                # positions on the 07:18 run of 2026-08-19. Capping the creep would fix the count and
                # break the placement (short by up to 14 cm against a +/-5 cm requirement), which is
                # why the trigger moved instead of the distance.
                emit_targets = _rest
                _err_m = travelled - _due          # how far past the target we actually stopped
                log("  emitter %d — punch reached (target %.2fm, stopped at %.2fm, %+.0fmm), "
                    "%d more queued"
                    % (_run["planted"] + 1, _due, travelled, _err_m * 1000, len(emit_targets)))

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
                ml_raw = emit.get("ml_value")
                log("  emitter %d — STOPPED at %.2fm (conf %.2f%s, y=%d/%d) [frame %s]%s"
                    % (_run["planted"] + 1, travelled, emit["confidence"],
                       "" if ml_raw is None else ", ml %.2f" % ml_raw, ey, h, shot,
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
    # NO bus.stop() HERE. It lives in _cam_loop's finally, so it also runs when this
    # function raises or returns early — which is the whole point of the split.

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
    _cam["url"] = url                # a changed url makes the old loop exit (while _cam["url"]==url)

    # WAIT FOR THE OLD LOOP TO ACTUALLY DIE before starting another.
    # This used to sleep 0.3s and hope. The loop can be blocked inside a socket read for
    # seconds, so the replacement started while the old one still held the stream — and the
    # ESP32-CAM serves ONE client, so the new connection timed out and took the working one
    # down with it (2026-08-18: "camera open failed: timed out", then 4 failed reopens and
    # the loop was dropped). Joining is bounded, so a wedged thread cannot block the UI.
    if alive and _cam_thread is not None:
        _cam_thread.join(timeout=6.0)
        if _cam_thread.is_alive():
            log("WARNING old camera loop did not exit in 6s — starting the new one anyway; "
                "if the stream serves a single client expect this connect to fail")
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
# The UI thumbnail is throttled INDEPENDENTLY of the save rate. Dataset capture can now run
# at the full frame rate (interval 0), and pushing a base64 thumbnail 30 times a second would
# flood the operator socket and stall the control loop inside send_message — the saving is the
# point, the preview is only reassurance that frames are arriving.
_CAP_THUMB_MAX_FPS = 2.0
_cap_thumb_last = [0.0]


def _save_capture(frame):
    """Save one RAW frame to disk; push a thumbnail to the UI at most _CAP_THUMB_MAX_FPS."""
    try:
        os.makedirs(_CAP_DIR, exist_ok=True)
        name = "cap_%s.jpg" % datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        cv2.imwrite(os.path.join(_CAP_DIR, name), frame)     # full-res, unannotated
        _capture["count"] += 1
        now = time.time()
        if now - _cap_thumb_last[0] < 1.0 / _CAP_THUMB_MAX_FPS:
            return
        _cap_thumb_last[0] = now
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
        # MILLISECONDS, not seconds. At 19fps several frames can be saved within one second —
        # the suspect screen alone trips on ~9% of frames — and a second-resolution name means
        # the later save SILENTLY overwrites the earlier one. Losing the frame that explains a
        # failure, quietly, is the worst outcome for a file whose only job is evidence.
        name = "%s_%s.jpg" % (tag, datetime.now().strftime("%H%M%S_%f")[:-3])
        cv2.imwrite(os.path.join(_CAP_DIR, name), frame)
        return name
    except Exception as e:                                    # noqa: BLE001
        print("named save failed: %s" % e, flush=True)
        return ""


def _fresh_frame(bus, budget_s=1.5):
    """A frame captured AFTER this call — never one buffered from before it.

    Replaces a heuristic that read frames until one took longer than 40ms and called that
    the live edge: it was guessing at the buffer depth, and it guessed wrong often enough
    that two alignment frames came back byte-for-byte identical in scene. The bus stamps
    each frame with its capture time, so "newer than now" is exact rather than inferred.
    """
    t, frame = bus.wait_fresh(after_t=time.time(), timeout=budget_s)
    return (frame is not None), frame


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
# A BOUND ON HOW FAR THE SEARCH MAY ROTATE THE ROBOT, added 2026-08-19.
#
# The nudges escalate (0.30 -> 0.48 -> 0.75s, x1.6 capped) and, before this, nothing limited their
# SUM. Seven iterations is 4.53s of motor-on, which at the measured 60-83 dps is 272-376 degrees —
# more than a full turn. The 02:33 run issued 137 degrees of uncommanded rotation this way and
# ended with the lateral running nearly horizontal in frame, which is the "turn ended at 45 deg"
# and "turned more than 180 deg" symptom reported across several days. The gyro was never wrong:
# the commanded pivot landed at err -0.0 deg and the SEARCH threw the heading away afterwards.
#
# 25 degrees is chosen as "more than any plausible landing error, far less than a lost heading".
# Past it the run stops and says so, because a bounded failure the operator can nudge in five
# seconds beats a robot at an arbitrary heading thrashing LOST/REGAINED ten times.
_NUDGE_MAX_TOTAL_DEG = 25.0
# Frames to sample per align decision, and how many must agree. A SINGLE frame is not enough: the
# align loop acted on one reading, and on 2026-08-19 that reading was bare soil reported at 3.4
# sigma (lat2_align2 — the saved frame contains no tube at all). That is the same false-positive
# rate measured offline on the crossing-frame negatives, 1 in 9 passing every gate. Two of three
# frames agreeing within a tube's width removes it, at a cost of ~150ms per decision.
_ALIGN_FRAMES    = 3
_ALIGN_AGREE     = 2
_ALIGN_AGREE_PX  = 25.0
# HOW FAR OFF CENTRE A TUBE MAY BE AND STILL BE THE ONE WE PIVOTED ON. See _align_is_ours: the
# previous lateral is a real, perfectly tube-shaped tube and the ONLY thing wrong with it is its
# position, so this is the only gate that can tell them apart. 70px is about 12.5cm of ground —
# wider than a plausible landing error, and far narrower than the ~279px the previous lateral sits
# at with a 0.50m row gap.
_ALIGN_CENTRE_PX = 70.0
_NUDGE_PULSE_S   = 0.30
_NUDGE_PULSE_MAX = 0.75
_NUDGE_GROWTH    = 1.6
# _NUDGE_TOL is gone with the centre-by-rotation loop it served: align no longer decides when a
# tube is "centred enough", it decides whether the tube is OURS and then hands over. Deciding
# centredness here is what declared success at "off +2 px" on a frame whose tube ran nearly
# horizontal, and then handed the follow loop a robot at the wrong heading.
_NUDGE_MAX       = 6


def _align_look(bus):
    """Sample _ALIGN_FRAMES fresh frames and return a tube only if enough of them AGREE.

    Returns (tube, frame, n_agree). `tube` is the median-x reading of the agreeing group, or None.

    Pure gates cannot do this job: lat2_align2 passed _tube_plausible (w=46, s=3.4) on a frame of
    bare soil. What separates a real tube from a strong soil artefact is that the real one is STILL
    THERE on the next frame, in the same place. This is the same reasoning as _track_tube's relock
    requiring its pending rejects to agree.
    """
    seen = []
    last_frame = None
    for _ in range(_ALIGN_FRAMES):
        ok, frame = _fresh_frame(bus)
        if not ok:
            continue
        last_frame = frame
        t = detect_tube(frame)
        if _tube_plausible(t):
            seen.append((t["tube_x"], t, frame))
    if len(seen) < _ALIGN_AGREE:
        return None, last_frame, len(seen)
    # largest cluster of readings within _ALIGN_AGREE_PX of each other
    best = []
    for x0, _t, _f in seen:
        grp = [s for s in seen if abs(s[0] - x0) <= _ALIGN_AGREE_PX]
        if len(grp) > len(best):
            best = grp
    if len(best) < _ALIGN_AGREE:
        return None, last_frame, len(best)
    best.sort(key=lambda s: s[0])
    mid = best[len(best) // 2]
    return mid[1], mid[2], len(best)


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
# RE-MEASURED 2026-08-18 for the USB camera, mounted 18cm high and tilted down:
#     bottom of frame -> wheel contact      23 cm
#     wheel contact   -> front axle        + 5 cm
#     front axle      -> pivot axis        +13 cm   (half the 26cm wheelbase)
#     bottom of frame -> PIVOT AXIS          41 cm
# Visible ground runs 23cm to 45cm ahead of the wheels (a 22cm strip).
#
# The +5 and +13 are the ones people forget: a skid-steer rotates about the MIDDLE of its
# wheelbase, not the wheel contact patch, so stopping "at the wheels" parks the lateral half
# a wheelbase behind the pivot point and the robot turns beside the row instead of onto it.
# (History: 0.20 was a pure guess; 0.38 was the ESP32-CAM at 15cm high seeing a 34cm strip.)
_ARRIVE_EXTRA_M = 0.41
_ARRIVE_MAX_S   = 6.0


def _creep(metres):
    """Blind forward creep of roughly this distance (open-loop, like _pivot)."""
    _drive(*trimmed(_BASE_PWM, _BASE_PWM), "drip", "forward")
    time.sleep(max(0.0, metres / max(0.01, _DRIP_SPEED_MPS)))
    _drive_stop("drip")
    time.sleep(0.3)


def _arrive_over_lateral(bus):
    """Creep until the lateral is at the wheels. Returns the best nearness reached."""
    deadline = time.time() + _ARRIVE_MAX_S
    best = 0.0
    while time.time() < deadline:
        _drive(*trimmed(_BASE_PWM, _BASE_PWM), "drip", "forward")
        _t, frame = bus.latest()
        if frame is None or bus.age() > bus.stale_after():
            time.sleep(0.01)          # never judge arrival on a stale view
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


def _align_look(grab, frames=None, agree=None, agree_px=None):
    """Sample several fresh frames and return a tube only if enough of them AGREE on its column.

    `grab` is a zero-arg callable returning (ok, frame) — injected so this is testable without a
    board. Returns (tube, frame, n_agree); `tube` is the median-column reading of the largest
    agreeing group, or None.

    WHY AGREEMENT AND NOT A BETTER GATE. lat2_align2 (a saved frame from the 2026-08-19 turn) is
    bare soil and straw with no tube in it at all, and detect_tube reported a tube at x=97 with
    strength 3.4 — passing every shape gate there is. What separates it from a real tube is that it
    is NOT THERE on the neighbouring frames: align1 and align3 both find nothing. Measured over that
    saved sequence, the false positive is isolated and two-of-three agreement removes it.
    Same reasoning as _track_tube's relock requiring its pending rejects to agree.
    """
    frames = _ALIGN_FRAMES if frames is None else frames
    agree = _ALIGN_AGREE if agree is None else agree
    agree_px = _ALIGN_AGREE_PX if agree_px is None else agree_px

    seen, last_frame = [], None
    for _ in range(frames):
        ok, frame = grab()
        if not ok or frame is None:
            continue
        last_frame = frame
        t = detect_tube(frame)
        if _tube_plausible(t):
            seen.append((float(t["tube_x"]), t, frame))
    if len(seen) < agree:
        return None, last_frame, len(seen)
    best = []
    for x0, _t, _f in seen:
        grp = [g for g in seen if abs(g[0] - x0) <= agree_px]
        if len(grp) > len(best):
            best = grp
    if len(best) < agree:
        return None, last_frame, len(best)
    best.sort(key=lambda g: g[0])
    mid = best[len(best) // 2]
    return mid[1], mid[2], len(best)


def _align_is_ours(tube, frame_w):
    """Is this the lateral we pivoted ON, or the one we came from?

    We drove until the crossing was under the pivot axis and then turned about that axis, so the
    target tube MUST be near the middle of the frame. Anything far off-centre is a different tube.
    That distinction cannot be made from confidence or shape — on 2026-08-19 the robot locked onto
    the PREVIOUS lateral during the turn and drove back down the row it had just finished. The old
    lateral is a real, perfectly tube-shaped tube; the only thing wrong with it is WHERE it is.

    Measured for the current geometry: at a 0.50m row gap the previous lateral sits about 279px from
    centre, against a 160px half-frame — just outside the view, and it sweeps through on any
    sideways slip. _ALIGN_CENTRE_PX = 70 (about 12.5cm of ground) is comfortably wider than a
    plausible landing error and far narrower than that 279px, so the two never overlap.
    """
    return abs(float(tube["tube_x"]) - frame_w / 2.0) <= _ALIGN_CENTRE_PX


def _turn_onto_tube(bus, turn_right, tag):
    """Pivot onto the lateral, then hand to the follow loop as soon as the tube is identified.

    REWRITTEN 2026-08-19. The previous version rotated in place to "centre" the tube, and that was
    wrong in three separate ways, all of which the field showed:

    1. IT COULD SPIN THE ROBOT MORE THAN A FULL TURN. The nudges escalated 0.30 -> 0.48 -> 0.75s and
       nothing bounded their sum: seven iterations is 4.53s of motor-on, which at the measured 60-83
       dps is 272-376 degrees. The 02:33 run spent 137 degrees this way and finished with the
       lateral running nearly horizontal in frame. The commanded pivot had landed at err -0.0 deg —
       the SEARCH threw the heading away, not the turn.

    2. IT ASSUMED "no tube visible" MEANT "under-turned" and always nudged the same way. After a
       pivot that skidded sideways on the plastic tube the cause is lateral displacement, and
       rotating further makes it worse.

    3. CENTRING BY ROTATION IS THE WRONG TOOL. A robot 10cm to the side but perfectly parallel gets
       rotated until the tube looks centred, which leaves it at an angle to the row: centred for one
       frame, then off the row. The follow loop corrects cross-track offset WHILE DRIVING, using the
       near and far corrections together, and does it better than pivoting in place can.

    So: search only while nothing is visible, bounded hard; identify the tube by POSITION so the
    neighbouring lateral cannot be mistaken for it; then hand over and drive. Returns True if a
    trusted tube was found.
    """
    _pivot(_TURN_ON_COARSE, turn_right)
    spent_deg = 0.0
    probe_right = turn_right
    pulse = _NUDGE_PULSE_S
    for i in range(_NUDGE_MAX + 1):
        t, frame, n_agree = _align_look(lambda: _fresh_frame(bus))
        if frame is None:
            log("  align: no frame after the turn — leaving it open-loop")
            return False
        name = _save_named(frame, "%s_align%d" % (tag, i))
        w = frame.shape[1]

        if t is not None and _align_is_ours(t, w):
            # DON'T LEAVE THE FOUND TUBE. Seed the tracker so the follow loop starts HINTED on this
            # column instead of cold: its jump gate (~30px) then keeps it here rather than letting
            # it wander onto the neighbouring lateral, which is exactly what happened before.
            _track["x"] = float(t["tube_x"])
            _track["last_ok"] = time.time()
            _track["rejects"], _track["pending"] = 0, []
            log("  align %d: tube at x=%.0f (%+.0fpx off centre), %d/%d frames agreed, %s — "
                "handing to the follow loop with the tracker seeded [%s]"
                % (i, t["tube_x"], t["tube_x"] - w / 2.0, n_agree, _ALIGN_FRAMES,
                   _tilt_str(t), name))
            return True

        if t is not None:
            # A real, tube-shaped reading in the wrong place. Almost certainly the lateral we came
            # from. Say so explicitly, because "found a tube" and "found OUR tube" reading the same
            # in the log is what hid this for days.
            log("  align %d: IGNORING tube at x=%.0f (%+.0fpx off centre, limit %.0f) — too far "
                "off to be the one we pivoted on; the previous lateral sits about 279px out [%s]"
                % (i, t["tube_x"], t["tube_x"] - w / 2.0, _ALIGN_CENTRE_PX, name))

        if spent_deg >= _NUDGE_MAX_TOTAL_DEG or i >= _NUDGE_MAX:
            _drive_stop("drip")
            log("  align: NO tube of ours after %d looks and %.0f deg of search — STOPPING rather "
                "than driving off at an unknown heading. The gyro closes the pivot angle, so this "
                "is lateral displacement: nudge the robot onto the lateral by hand and the follow "
                "loop will pick it up. [%s]" % (i + 1, spent_deg, name))
            return False

        step = min(pulse, max(0.05, (_NUDGE_MAX_TOTAL_DEG - spent_deg) / max(1.0, CAL["tdps"])))
        log("  align %d: nothing of ours (%d/%d frames agreed) — probing %s %.2fs, %.0f of %.0f "
            "deg spent [%s]"
            % (i, n_agree, _ALIGN_FRAMES, "right" if probe_right else "left", step,
               spent_deg, _NUDGE_MAX_TOTAL_DEG, name))
        _nudge(probe_right, step)
        spent_deg += step * CAL["tdps"]
        probe_right = not probe_right       # ALTERNATE: covers over-turn and sideways slip too
        pulse = min(_NUDGE_PULSE_MAX, pulse * _NUDGE_GROWTH)
    return False

def _emit_capture_status():
    ui.send_message("capture_status", {
        "on": _capture["on"], "count": _capture["count"],
        "interval": _capture["interval"], "dir": _CAP_DIR,
        "cam_connected": bool(_cam["url"]),
    })

def on_capture_start(client, data):
    # FLOOR LOWERED FROM 0.3s TO 0. `interval=0` means SAVE EVERY FRAME, which two jobs need
    # and neither could get before:
    #   * the emitter training set — at 2s the robot moves 34cm between saves and steps clean
    #     over most emitters; at frame rate each one is caught 4-6 times at different
    #     distances and scales, which is exactly the variety FOMO needs
    #   * temporal filtering (accepting a weaker per-frame detector and voting across frames)
    #     cannot be validated at all without CONSECUTIVE frames. Every frame set we owned was
    #     2s apart, so there was nothing to test it against.
    # ~18KB a frame at 30fps is 32MB/minute against 15GB free, so a few one-minute passes are
    # nothing. Subsample before uploading to Edge Impulse — near-duplicate frames leak between
    # the train and test split and make the accuracy figure meaningless.
    _capture["interval"] = max(0.0, float(data.get("interval", 2.0)))
    _capture["last"] = 0.0            # capture the next frame immediately
    _capture["on"] = True
    os.makedirs(_CAP_DIR, exist_ok=True)
    print("capture: ON (every %.1fs) -> %s" % (_capture["interval"], _CAP_DIR), flush=True)
    _emit_capture_status()

def on_capture_interval(client, data):
    """Set the capture interval LIVE, whether or not capture is currently running.

    Without this the interval was read in exactly one place — on_capture_start — and every
    other route into capture used whatever value happened to be left in _capture. That cost a
    whole dataset on 2026-08-18: the operator moved the slider to 0.5s and pressed
    "Follow tube & capture", which is on_run_start (scan mode), which turns capture on WITHOUT
    consulting the slider. So 127 frames were saved at the 2.0s default — 34cm apart, stepping
    clean over most emitters, which is the exact problem the 0.5s was meant to fix — and
    nothing in the UI or the log contradicted the operator's belief that it was 0.5s.

    A control that silently does nothing is worse than a missing control.
    """
    _capture["interval"] = max(0.0, float(data.get("interval", 2.0)))
    print("capture: interval set to %.2fs%s" % (
        _capture["interval"], "" if _capture["on"] else " (capture is off)"), flush=True)
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
    # edge_margin_m=0 SO THE ROBOT'S STARTING POSITION IS THE FIRST SEED.
    #
    # SeedPlan defaults the inset to HALF the relevant gap, which centres seeds in their cells and
    # keeps the outer row off the boundary. Geometrically tidy, but wrong for how this is actually
    # operated: the operator physically places the robot at corner 1 and expects it to plant where
    # it stands. With the default the first spot sat 50cm across and 20cm along from corner 1,
    # which reads as the robot ignoring where you put it.
    #
    # THIS ALSO CHANGES THE SPOT COUNTS, because _count() is int((span - 2*inset)/gap) + 1:
    #     inset=gap/2 : w=2.0 gap=1.0 -> 2 rows at 0.5, 1.5   l=5.0 gap=0.4 -> 12 seeds 0.2..4.6
    #     inset=0     : w=2.0 gap=1.0 -> 3 rows at 0, 1, 2    l=5.0 gap=0.4 -> 13 seeds 0.0..4.8
    # So `w` now means the distance between the OUTERMOST rows, not a bounding box with margins:
    # 2 rows 1m apart is w=1.0, not w=2.0. Worth knowing before entering numbers.
    #
    # The trade given up: the outermost row now sits ON the plot boundary, so there is no built-in
    # headland for the turn. The turn happens at the far end of the row either way, so it needs
    # clear ground past corner 2 rather than inside the marked plot.
    return SeedPlan(plot_w_m=_plot["w"], plot_l_m=_plot["l"],
                    row_gap_m=_plot["row_gap"], seed_gap_m=_plot["seed_gap"],
                    seeds_per_spot=int(_plot["seeds_per_spot"]),
                    edge_margin_m=0.0,
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
    # preview=True IS LOAD-BEARING. Without it this message carries no `planned` key, and the
    # UI's draw() rebuilds the whole SVG from whatever arrived — so `d.planned || []` renders the
    # corner markers and nothing else. _emit_run() is called from 20 places (every state change,
    # every plant, every plot_mark), so the plan appeared on plot_config and was then WIPED by the
    # next corner mark. That is exactly what it did on 2026-08-18: four green marks, four corner
    # circles, no dots, and the placeholder text back.
    #
    # The comment here used to read "keep the plan overlay in step", which is what it failed to
    # do. Recomputing the preview is arithmetic over a couple of dozen waypoints, so there is no
    # reason to send a partial message the client can only read as "no plan".
    _emit_plot(preview=True)                       # keep the plan overlay in step


# ── Heading drift on plain-land hops ─────────────────────────────────────────
# PLAIN-LAND ONLY. Drip mode follows the tube by sight, which is closed-loop on the real target;
# holding a heading there would fight the vision steering on a lateral that genuinely curves.
#
# A plot row is 13 open-loop hops against one fixed trim. A 2-3 deg veer per hop is 2cm sideways
# — nothing once, 39 deg of heading error by the end of a row, and nothing measured it.
#
# MEASURE-ONLY UNTIL THE SIGN IS CONFIRMED. The correction direction depends on whether a LEFT
# (counter-clockwise) rotation reads POSITIVE or negative from this gyro, and that cannot be read
# off the firmware: pivotDeg only ever compares fabsf(signed_a), so it never reveals the
# convention. Guessing it would DOUBLE the error instead of cancelling it, on a robot that is
# already hard to keep on a row. So the first runs log the drift and act on nothing.
#
# TO CONFIRM IT, with the robot switched on and motors idle:
#     ssh unoq 'docker exec motor-control-main-1 python3 /app/python/field_test.py gspin 5'
#   then turn the robot BY HAND anti-clockwise through most of a turn while it counts.
#   gyroIntegrate reports SIGNED degrees, so the sign it prints IS the convention:
#     positive -> anti-clockwise (left) is positive  -> set _YAW_LEFT_POSITIVE = True
#     negative -> anti-clockwise (left) is negative  -> set _YAW_LEFT_POSITIVE = False
#   Then set _HEADING_CORRECT = True.
#
# BACK OFF TO MEASURE-ONLY, 2026-08-19. The sign is confirmed and the plumbing works, but the
# gyro has a SYSTEMATIC BIAS the firmware's per-hop estimate does not remove, so the corrector
# would be integrating fiction. Measured with five STATIONARY yawHop calls at the real plot hop
# length (2121 ms = startup -0.303s + 0.40m / 0.165 m/s) and ZERO PWM, so every degree below is
# phantom:
#
#     +0.04, +0.41, +0.29, +0.44, +0.43   ->  +1.61 deg over 5 hops = +0.32 deg/hop
#
# ALL THE SAME SIGN, so it accumulates instead of cancelling. The firmware reported bias -0.21
# dps on each call and still left ~+0.15 dps behind. A 13-hop row is +4.2 deg, which stays under
# the 10 deg threshold — but _plot_yaw["err"] accumulates over the WHOLE run, and a 6-row run is
# ~78 hops = ~25 deg of phantom drift. That is two or three spurious corrections, each pivoting
# the robot up to 10 deg the WRONG way, on a robot whose veer we are trying to measure.
#
# So: log every hop, correct nothing. One dry run then gives real per-hop yaw WHILE DRIVING, which
# can be compared against the +0.32 deg/hop phantom above to separate bias from true veer. Only
# then is there a defensible correction — either a per-run tare (measure the phantom at run start
# and subtract rate * hop_seconds) or a firmware fix to the bias window.
#
# RE-ENABLED 2026-08-19, after the measure-only dry run supplied exactly that characterisation:
#
#     26 hops, +18.2 deg total apparent drift          = +0.70 deg/hop while driving
#     stationary bias, +0.151 dps over a 3.33s hop     = +0.50 deg/hop
#     => real veer                                      = +0.20 deg/hop
#
# MIND THE HOP LENGTH. The stationary samples were taken at 2121ms (a 0.40m hop) and read
# +0.32 deg each, but that run used 0.60m hops = 3333ms, where the same +0.151 dps is +0.50 deg.
# Comparing the two directly says "bias is half the drift"; scaling properly says it is 71% of it,
# and the real veer is +0.20 deg/hop rather than +0.38. Always convert through the RATE.
#
# Either way, correcting the raw figure would have over-corrected by ~3.5x. The bias is now removed
# per run by _plot_yaw_tare(), and this flag only takes effect once that tare has SUCCEEDED — see
# the guard at the correction site, which keeps a run measure-only by itself if the tare is refused.
#
# The residual veer is still worth correcting, because it accumulates: the 2026-08-19 run was
# +6.1 deg off heading by the first row change, and through row 2 that put the robot ~71cm sideways
# of where it should have been against a 50cm row gap — which is why row 2 came back over row 1 to
# marker 1. The turns were never the problem: worst pivot error that run was 1.7 deg.
_HEADING_CORRECT = True           # sign confirmed 2026-08-18; bias tared per run since 2026-08-19
# CONFIRMED 2026-08-18 with the flashed firmware, which is better than the hand-spin because it
# exercises the real path: yawHop(120, -120, 800) commands (+L,-R) = a RIGHT pivot, and the gyro
# reported yaw_deg = -60.26. So a right turn reads NEGATIVE and a left turn reads POSITIVE.
_YAW_LEFT_POSITIVE = True
# The threshold cannot be small: the learned pivot coast at turn_pwm is ~4-8 deg, so any pivot
# under about that simply releases and coasts straight past the target. 10 deg is the smallest
# correction the hardware can actually deliver. At 2-3 deg of veer per hop that is a correction
# roughly every four hops — visible, but nothing like 13 hops of silent accumulation.
_HEADING_ERR_LIMIT_DEG = 10.0
_plot_yaw = {"err": 0.0, "hops": 0, "worst": 0.0, "corrections": 0,
             "bias_dps": 0.0, "tared": False, "suspect": 0}

# How many stationary hops to average for the tare. The WINDOW is derived from the run's own hop
# length (see _plot_yaw_tare) rather than fixed: the bias is applied as rate x hop_seconds, so
# measuring it over the same interval it will be applied to is both the best signal-to-noise and
# the least room for an arithmetic slip. Getting that conversion wrong is not hypothetical — it
# made a 0.32 deg/hop measurement read as the bias for 0.60m hops when the true figure was
# 0.50 deg/hop, and briefly put the real veer at nearly twice its actual value.
#
# 4 hops at the demo's 0.60m spacing is 4 x 3.33s = 13s of standing still before the wheels move.
# That is the price of a correction that is not acting on fiction.
_TARE_HOPS = 4
_TARE_MS = 3000                   # fallback only, when the hop length is not known
_TARE_MS_MIN, _TARE_MS_MAX = 1500, 3500
# Refuse to tare if the samples disagree by more than this (deg/s). A consistent bias is what we
# can subtract; a noisy one means the sensor is not behaving and subtracting its mean would inject
# error rather than remove it.
_TARE_MAX_SPREAD_DPS = 0.30


def _plot_yaw_reset():
    _plot_yaw.update(err=0.0, hops=0, worst=0.0, corrections=0,
                     bias_dps=0.0, tared=False, suspect=0)


def _plot_yaw_tare(hop_ms=None):
    """MEASURE the gyro's stationary drift rate and store it, so hops can be corrected for it.

    `hop_ms` is the duration of the run's real forward hop. Passing it makes each tare sample the
    same length as the hops the bias will be subtracted from, which is the whole point.

    WHY. Measured 2026-08-19 on five stationary yawHop calls at zero PWM, real hop length:
    +0.04, +0.41, +0.29, +0.44, +0.43 deg -> +0.32 deg/hop, ALL THE SAME SIGN. The firmware's own
    per-hop bias estimate reported -0.21 dps each time and still left ~+0.15 dps behind, because
    its sampling window is too short to separate a slow drift from the hop's real rotation.

    That bias is not a rounding detail. Over the 26-hop dry run of 2026-08-19 the robot accumulated
    +18.2 deg of apparent drift; the stationary rate accounts for +8.3 deg of it, i.e. MORE THAN
    HALF. Correcting the raw number would have over-corrected by roughly 2x and bent the run in the
    opposite direction, confidently.

    A tare rather than a constant, because it is measured with the robot on the ground it is about
    to run on, at the temperature it is actually at — a gyro's zero-rate offset moves with both.

    This is deliberately NOT a substitute for fixing the firmware's bias window; it is the honest
    thing to do from Python without another flash, and it is measured every run so it cannot go
    stale the way a hard-coded offset would.
    """
    if not _gyro_ready():
        log("heading tare: no gyro — correction stays off for this run")
        return
    ms = int(min(_TARE_MS_MAX, max(_TARE_MS_MIN, hop_ms or _TARE_MS)))
    log("heading tare: measuring stationary drift, %d x %dms (%.0fs total) — "
        "DO NOT MOVE THE ROBOT" % (_TARE_HOPS, ms, _TARE_HOPS * ms / 1000.0))
    rates, rejects = [], 0
    for i in range(_TARE_HOPS):
        try:
            r = json.loads(_decode(Bridge.call("yawHop", 0, 0, ms)))
        except Exception as e:                                   # noqa: BLE001
            log("heading tare: yawHop failed (%s) — correction stays off" % e)
            return
        if not r.get("ok"):
            log("heading tare: yawHop said %s — correction stays off" % r.get("err", "?"))
            return
        rejects += int(r.get("rejected", 0))
        rates.append(float(r.get("yaw_deg", 0.0)) / (ms / 1000.0))
        log("  tare %d/%d: %+.3f deg over %dms = %+.3f dps (rejected %d)"
            % (i + 1, _TARE_HOPS, r.get("yaw_deg", 0.0), ms, rates[-1],
               r.get("rejected", 0)))
    spread = max(rates) - min(rates)
    mean = sum(rates) / len(rates)
    if spread > _TARE_MAX_SPREAD_DPS:
        log("heading tare: REFUSED — samples spread %.3f dps (limit %.3f), mean %+.3f. A bias "
            "this noisy is not a bias; subtracting it would add error. Correction stays off and "
            "this run is measure-only." % (spread, _TARE_MAX_SPREAD_DPS, mean))
        return
    _plot_yaw["bias_dps"] = mean
    _plot_yaw["tared"] = True
    log("heading tare: %+.3f dps (spread %.3f, %d spike samples rejected) = %+.2f deg per %dms "
        "hop. Every hop's yaw now has bias x hop_seconds removed; correction is %s."
        % (mean, spread, rejects, mean * ms / 1000.0, ms,
           "ARMED" if _HEADING_CORRECT else "still off by config"))


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
        """Drive one hop and MEASURE the rotation, so veer stops accumulating unseen.

        Falls back to the base timed hop whenever the gyro is missing or yawHop fails, so this is
        never worse than what it replaces.

        The battery gain that BridgeRobot._apply() would normally apply is skipped: the monitor is
        disabled (BATT_PRESENT 0, since A4/A5 carry the gyro) so the gain is 1.0 anyway. If the
        probe is ever refitted this must be restored, or the duty will be wrong under sag.
        """
        self._gate()
        if not _gyro_ready():
            super().forward(distance_m)
            return
        l = int(round(self.pwm * self.left_trim))
        r = int(round(self.pwm * self.right_trim))
        ms = int(1000 * max(0.0, self.startup_s + distance_m / self.speed_mps))
        try:
            res = json.loads(_decode(Bridge.call("yawHop", l, r, ms)))
        except Exception as e:                                  # noqa: BLE001
            log("  yawHop RPC failed (%s) — plain timed hop, no heading measurement" % e)
            super().forward(distance_m)
            return
        if not res.get("ok"):
            log("  yawHop: %s — plain timed hop" % res.get("err", "?"))
            super().forward(distance_m)
            return

        # Dead reckoning on the COMMANDED move, exactly as the base class does. The measured
        # rotation is deliberately NOT fed back into x/y: it says the heading drifted, not where
        # the robot ended up, and pretending otherwise would corrupt the position estimate too.
        rad = math.radians(self.heading)
        self.x += distance_m * math.cos(rad)
        self.y += distance_m * math.sin(rad)

        # THE MCU DRIVES THE MOTORS ITSELF for yawHop, bypassing _apply(), so self._sent was never
        # updated and _check_diag compared the MCU's real duty against a stale (0, 0). That
        # produced 30 bogus "MCU received 46/55 but we sent 0/0" warnings in the 2026-08-19 run —
        # one per hop and turn — which is worse than useless: it buries a GENUINE mismatch under
        # noise. Record what we actually commanded.
        self._sent = (l, r)

        yaw_raw = float(res.get("yaw_deg", 0.0))
        # SUBTRACT THE MEASURED STATIONARY DRIFT. bias_dps is 0.0 until _plot_yaw_tare() succeeds,
        # so an untared run behaves exactly as before rather than half-correcting.
        drift = _plot_yaw["bias_dps"] * (ms / 1000.0)
        yaw = yaw_raw - drift

        # A HOP WHOSE SAMPLES WERE MOSTLY THROWN AWAY IS NOT A MEASUREMENT OF ZERO. Hop 14 of the
        # 2026-08-19 run came back `peak -186 dps, rejected 19, yaw +0.0`: the spike filter had
        # discarded real rotation along with the artefact, and +0.0 read as "did not turn" when it
        # meant "we do not know". It is still accumulated — it remains the best estimate available
        # and discarding it would leave a silent hole in the drift total — but it is marked here
        # and counted in the scorecard, so a recurring sensor fault shows up as a number instead of
        # hiding inside a plausible-looking zero.
        rej = int(res.get("rejected", 0))
        suspect = rej > 5
        if suspect:
            _plot_yaw["suspect"] += 1

        _plot_yaw["err"] += yaw
        _plot_yaw["hops"] += 1
        if abs(yaw) > abs(_plot_yaw["worst"]):
            _plot_yaw["worst"] = yaw
        self._snap("fwd %.2fm yaw %+.1f" % (distance_m, yaw))

        # LOG EVERY HOP, not just the ones that trigger a correction. Without this a CLEAN run
        # produces no heading data at all, so "the correction works" and "the correction never
        # ran" look identical afterwards — and the clean run is the one that has to prove it.
        # `rejected` is the MCU's spike filter (spurious ~-186 dps samples seen while stationary);
        # if it climbs above 1-2 per hop the gyro read path itself needs looking at, and nothing
        # else in the system would ever tell us. `judder` is a magnitude and must never be < 0.
        log("  hop %2d: %.2fm  yaw %+5.1f  cum %+6.1f  (raw %+5.1f - drift %+.2f | peak %+.0f "
            "dps, judder %.1f, rejected %d, bias %+.3f)%s"
            % (_plot_yaw["hops"], distance_m, yaw, _plot_yaw["err"], yaw_raw, drift,
               res.get("peak_dps", 0.0), res.get("judder", 0.0), rej,
               res.get("bias", 0.0),
               "  <- SUSPECT: %d samples rejected, yaw unreliable" % rej if suspect else ""))

        e = _plot_yaw["err"]
        if abs(e) < _HEADING_ERR_LIMIT_DEG:
            return
        # THREE CONDITIONS, and the tare is the one that was missing. Without it the corrector
        # integrates the gyro's stationary bias, which on 2026-08-19 was more than half the
        # apparent drift — so it would have pivoted the robot on fiction. If the tare was refused
        # (noisy samples, no gyro, RPC failure) the run stays measure-only BY ITSELF rather than
        # needing someone to remember to switch it off.
        if not (_HEADING_CORRECT and _YAW_LEFT_POSITIVE is not None and _plot_yaw["tared"]):
            log("  HEADING DRIFT %+.1f deg over %d hops (worst hop %+.1f) — NOT corrected (%s)"
                % (e, _plot_yaw["hops"], _plot_yaw["worst"],
                   "no tare, so the bias is unknown" if not _plot_yaw["tared"]
                   else "correction disabled by config"))
            _plot_yaw["err"] = 0.0          # do not let it grow without bound in the log
            return
        # drifted one way -> pivot the other. _pivot takes (magnitude, turn_right).
        drifted_left = (e > 0) if _YAW_LEFT_POSITIVE else (e < 0)
        log("  heading drift %+.1f deg over %d hops — correcting %s"
            % (e, _plot_yaw["hops"], "right" if drifted_left else "left"))
        _pivot(abs(e), drifted_left)
        _plot_yaw["err"] = 0.0
        _plot_yaw["corrections"] += 1

    def turn_to(self, heading_deg):
        """GYRO-CLOSED turn, same as the drip row change. Falls back to the timed pivot.

        Plot mode used BridgeRobot.turn_to, which computes a DURATION from tdps/tstartup and
        drives blind for that long — so skid becomes heading error and it ACCUMULATES row over
        row. That is exactly what the gyro was fitted to fix, and until now plain-land runs got
        none of the benefit: measured over four consecutive 90 deg turns, the gyro pivot's worst
        error is 0.3 deg with 355 deg of real rotation for 360 commanded, i.e. no accumulation.

        _pivot() falls back to _pivot_timed() by itself on every failure path — no gyro, RPC
        error, or an MCU that reports not-ok — so this is strictly better than what it replaces
        rather than a new dependency. Two things it cannot do: turns smaller than the learned
        coast at turn_pwm (~8 deg), which is why the 0.5 deg deadband below stays; and freeing a
        physically stuck wheel, where it times out and SAYS so instead of driving off at the
        wrong heading.

        The heading is set from the COMMANDED angle, not a measured one. The MCU closes the loop
        internally and does not report the achieved angle back through pivotDeg, so the executor's
        dead-reckoning model still assumes the turn landed. That is a real limitation, and it is
        far smaller than the error the timed turn was leaving behind.
        """
        self._gate()
        delta = ((heading_deg - self.heading + 180) % 360) - 180   # shortest signed turn
        if abs(delta) <= 0.5:
            self.heading = heading_deg
            return
        # setMotors(-pwm, +pwm) swings CCW and INCREASES heading, so a positive delta is a LEFT
        # turn. Getting this backwards would send every row change the wrong way.
        turn_right = delta < 0
        _pivot(abs(delta), turn_right)
        self.heading = heading_deg
        # Same reason as in forward(): pivotDeg drives the motors on the MCU, so record the duty
        # the MCU will report or _check_diag flags every single turn as a mismatch. _pivot_timed
        # uses (p, -p) for a right turn, and the MCU log confirms 120/-120.
        p = int(self.turn_pwm)
        self._sent = (p, -p) if turn_right else (-p, p)
        self._snap("gyro turn %+.0fdeg" % delta)

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
    # A second run must not inherit the first run's accumulated drift — the same class of bug
    # that made a second run resume the previous run's `travelled`.
    _plot_yaw_reset()
    # TARE BEFORE THE WHEELS TURN. The robot is stationary here by definition (the operator has
    # just placed it on the first seed spot and pressed Start), which is the only moment a
    # stationary drift rate can be measured without stopping the run to do it.
    #
    # Measured over the SAME duration as the run's real hops, computed from the configured seed
    # spacing exactly as _ProgressRobot.forward does it.
    _hop_ms = 1000 * max(0.0, CAL["startup"] + float(_plot["seed_gap"]) / CAL["speed"])
    _plot_yaw_tare(hop_ms=_hop_ms)
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
        # HEADING SCORECARD — one line that says whether the gyro hop measurement earned its
        # keep. Printed on abort and on failure too (it is in `finally`), because a run that
        # went wrong is exactly when the drift history matters most.
        if _plot_yaw["hops"]:
            log("heading: %d hops measured, residual %+.1f deg, worst single hop %+.1f, "
                "%d correction(s) applied | tare %s%s%s"
                % (_plot_yaw["hops"], _plot_yaw["err"], _plot_yaw["worst"],
                   _plot_yaw["corrections"],
                   "%+.3f dps" % _plot_yaw["bias_dps"] if _plot_yaw["tared"] else "REFUSED",
                   " | %d suspect hop(s) (>5 gyro samples rejected)" % _plot_yaw["suspect"]
                   if _plot_yaw["suspect"] else "",
                   "" if _plot_yaw["tared"] else " — run was MEASURE-ONLY"))
        else:
            log("heading: NO hops measured — yawHop never ran (no gyro, or every call "
                "fell back to the timed hop). Veer was not being watched this run.")
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
ui.on_message("capture_interval", logged("capture_interval", on_capture_interval))
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
