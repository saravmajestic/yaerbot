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
    from vision.vision import detect_tube, detect_emitter, draw_overlay
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
LEFT_TRIM  = 0.77
RIGHT_TRIM = 1.00

def trimmed(left, right):
    return int(left * LEFT_TRIM), int(right * RIGHT_TRIM)


# Calibration measured 2026-08-11 (hard floor, 3S ~12V). These are surface- and
# hardware-specific — recalibrate after ANY mechanical/wiring change with
# scripts/field_test.py (solve/tsolve). See docs/farm-os/drive-precision.md.
# batt_comp stays off: the A4 divider still reads low and wanders.
CAL = {
    "pwm": 180, "speed": 0.616, "startup": 0.104,
    "ltrim": 0.75, "rtrim": 1.00,
    "turn_pwm": 120, "tdps": 51.0, "tstartup": -0.75, "tramp": 0.0,
}


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
    "left":     lambda s: trimmed(-s,  s),
    "right":    lambda s: trimmed( s, -s),
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
_cam = {"url": "", "w": 0, "h": 0, "tube": None, "emitter": None, "watch": 0.0}
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
_drip = {"emitter_gap": 0.40, "angles": [0, 90]}
_ARM_DWELL_S  = 0.6      # let the spool servo reach the angle before dropping
_MIN_REPLANT_M = 0.08    # anti-flicker floor only; NOT the emitter spacing
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
_BASE_PWM, _STEER_GAIN, _EMIT_CONF, _EMIT_COOLDOWN = 77, 10, 0.55, 3.0
# Tube-following creeps at _BASE_PWM, well below the PWM the drive was calibrated
# at, so scale the measured cruise speed by the duty ratio for the distance guard.
# Rough on purpose: it only has to tell 'same emitter' from 'the next one'.
_DRIP_SPEED_MPS = CAL["speed"] * (_BASE_PWM / CAL["pwm"])

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
    while _cam["url"] == url and _VISION_OK:
        ok, frame = src.read()
        if not ok:
            time.sleep(0.1)
            continue
        h, w = frame.shape[:2]
        tube = detect_tube(frame)
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

        # Tube found/lost EDGE logging. Logging every frame would flood at frame rate,
        # but logging nothing left "the robot moved 5cm and stopped" with no trace at
        # all in the log (2026-08-15) — the stall was invisible. So: log the two
        # transitions only, and only while a follow-run is active, otherwise an idle
        # camera would chatter every time something passes in front of it.
        _following = _run["state"] == "running" and _run["mode"] in ("scan", "drip")
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
                c = tube["correction"]
                l, r = trimmed(int(_BASE_PWM + _STEER_GAIN * c),
                               int(_BASE_PWM - _STEER_GAIN * c))
                _drive(max(0, min(255, l)), max(0, min(255, r)), "scan", "forward")
                if driving_since:
                    travelled += (now - driving_since) * _DRIP_SPEED_MPS
                driving_since = now
            else:
                _drive_stop("scan")          # lost the tube — don't drive blind
                driving_since = None

        elif _run["mode"] == "drip" and _run["state"] == "running":
            detected = emit["detected"] and emit["confidence"] >= _EMIT_CONF
            # Debounce on the DETECTION EDGE, not on an assumed spacing: the model is
            # what finds emitters, so "this one again" means "still in view". Re-arm
            # once it clears. _MIN_REPLANT_M is only a floor against flicker, well
            # below any real spacing — the operator's spacing figure never gates a
            # plant, so a missed emitter just means the next one is found normally.
            if detected and armed and travelled - last_plant_at >= _MIN_REPLANT_M:
                _drive_stop("drip")
                # One stop, then plant at EVERY selected arm position: [0, 90] gives
                # a 4-seed cross (0/180 then 90/270).
                for a in _drip["angles"]:
                    if _drip_rotates():
                        Bridge.call("indexSpool", int(a))
                        time.sleep(_ARM_DWELL_S)      # servo must arrive before the drop
                    if not _run["dry"]:
                        Bridge.call("plantSeed")      # both outlets: 2 seeds, 180 apart
                if _drip_rotates():
                    Bridge.call("indexSpool", 0)      # leave the arm flat for driving
                _run["planted"] += 1
                _run["positions"].append((0.0, round(travelled, 3)))
                armed = False
                last_plant_at = travelled
                _emit_run()
            elif not detected:
                armed = True                          # emitter left view — ready for the next

            # End of lateral is the MARKED plot length, so a long real lateral can be
            # demoed over a deliberately shorter marked plot.
            if travelled >= _plot["l"]:
                _drive_stop("drip")
                _run["state"] = "done"
                _run["msg"] = ("reached the marked plot length (%.1f m) — %d emitters"
                               % (_plot["l"], _run["planted"]))
                _emit_report(_drip_runlog())
                _emit_run()
            elif not detected or not armed:
                if tube["found"]:
                    c = tube["correction"]
                    l, r = trimmed(int(_BASE_PWM + _STEER_GAIN * c),
                                   int(_BASE_PWM - _STEER_GAIN * c))
                    _drive(max(0, min(255, l)), max(0, min(255, r)), "drip", "forward")
                    if driving_since:                 # integrate travel while moving
                        travelled += (now - driving_since) * _DRIP_SPEED_MPS
                    driving_since = now
                else:
                    _drive_stop("drip")               # lost the tube — don't drive blind
                    driving_since = None
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
    if url == _cam["url"] and _cam_thread and _cam_thread.is_alive():
        return                       # already streaming this url — don't spawn a duplicate loop
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
    "row_gap": 0.40, "seed_gap": 0.40, "seeds_per_spot": 1,
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
