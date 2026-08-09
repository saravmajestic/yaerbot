import base64
import json
import os
import sqlite3
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
    # the container's default gateway IS the UNO Q host
    try:
        out = subprocess.check_output(
            "ip route | awk '/default/{print $3; exit}'", shell=True
        )
        return out.decode().strip() or "172.19.0.1"
    except Exception:
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
        Bridge.call("setMotors", left, right)


def on_motor_stop(client, data):
    Bridge.call("stop")


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


# ── Seeder: space-based run (fixed arm) ─────────────────────────────────────
# Timed dead-reckoning: drive at a set speed for `gap_ms` (≈ the spacing), stop,
# plant, repeat. LIMITATION: distance = speed × time, so slopes / wheel-slip
# drift the real spacing. Good enough on flat ground; encoders would fix it.
# Drip-based + rotating-arm come later.
_seed = {"state": "idle", "phase": "—", "planted": 0, "seeds": 0}
_seed_cfg = {"gap_ms": 2000, "seeds_per_spot": 1, "drive_speed": 150}
_seed_thread = None

def _drive_gap():
    """Drive forward for gap_ms; abort only on STOP (state->idle). Returns False if stopped."""
    spd = _seed_cfg["drive_speed"]
    l, r = trimmed(spd, spd)
    Bridge.call("setMotors", l, r)
    end = time.time() + _seed_cfg["gap_ms"] / 1000.0
    while time.time() < end:
        if _seed["state"] == "idle":
            Bridge.call("stop")
            return False
        time.sleep(0.05)
    Bridge.call("stop")
    return True

def _seed_loop():
    try:
        while _seed["state"] != "idle":
            if _seed["state"] == "paused":      # pause takes effect between cycles
                time.sleep(0.1)
                continue
            _seed["phase"] = "driving"
            if not _drive_gap():
                break                            # stopped mid-drive
            if _seed["state"] == "idle":
                break
            _seed["phase"] = "planting"
            for _ in range(_seed_cfg["seeds_per_spot"]):
                if _seed["state"] == "idle":
                    break
                Bridge.call("plantSeed")
                _seed["seeds"] += 1
            _seed["planted"] += 1
    finally:
        Bridge.call("stop")
        _seed["phase"] = "—"

def _apply_seed_cfg(data):
    _seed_cfg["gap_ms"]         = max(200, int(data.get("gap_ms", _seed_cfg["gap_ms"])))
    _seed_cfg["seeds_per_spot"] = max(1, min(5, int(data.get("seeds_per_spot", _seed_cfg["seeds_per_spot"]))))
    _seed_cfg["drive_speed"]    = max(60, min(255, int(data.get("drive_speed", _seed_cfg["drive_speed"]))))

def on_seed_start(client, data):
    global _seed_thread
    _apply_seed_cfg(data)
    if _seed["state"] == "running":
        return
    if _seed["state"] == "paused":              # resume
        _seed["state"] = "running"
        return
    _seed.update(state="running", phase="driving", planted=0, seeds=0)
    _seed_thread = threading.Thread(target=_seed_loop, daemon=True)
    _seed_thread.start()

def on_seed_pause(client, data):
    if _seed["state"] == "running":
        _seed["state"] = "paused"
        Bridge.call("stop")

def on_seed_stop(client, data):
    _seed["state"] = "idle"
    Bridge.call("stop")

def on_plant_once(client, data):
    """Manual single plant (test button) — only when a run isn't active."""
    if _seed["state"] in ("idle",):
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
_drip = {"state": "idle"}          # idle | following
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

def _moisture_min():
    try:
        m = json.loads(_decode(Bridge.call("getMoisture")))
        return min(m.get("a0", 16383), m.get("a1", 16383))
    except Exception:
        return None

def _stream_url(base):
    """Accept 'ip', 'http://ip', or a full URL -> the MJPEG stream URL."""
    b = base.strip()
    if not b:
        return ""
    if b.startswith("http") and "/stream" in b:
        return b
    if b.startswith("http"):
        return b.rstrip("/") + ":81/stream"
    return "http://%s:81/stream" % b

def _cam_loop(url):
    last_plant = 0.0
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

        if _drip["state"] == "following":
            if emit["detected"] and emit["confidence"] >= _EMIT_CONF \
                    and (time.time() - last_plant) > _EMIT_COOLDOWN:
                Bridge.call("stop")
                Bridge.call("plantSeed")
                last_plant = time.time()
            elif tube["found"]:
                c = tube["correction"]
                l, r = trimmed(int(_BASE_PWM + _STEER_GAIN * c),
                               int(_BASE_PWM - _STEER_GAIN * c))
                Bridge.call("setMotors", max(0, min(255, l)), max(0, min(255, r)))
            else:
                Bridge.call("stop")

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
                        "drip": _drip["state"],
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
        "drip": _drip["state"],
    })

def on_drip_start(client, data):
    if _VISION_OK and _cam["url"]:
        _drip["state"] = "following"

def on_drip_stop(client, data):
    _drip["state"] = "idle"
    Bridge.call("stop")

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

def on_get_stats(client, data):
    batt_pct, batt_v = _battery()
    ui.send_message("stats", {
        "battery":   batt_pct,             # % from the getBattery RPC (None = no divider/RPC)
        "battery_v": batt_v,               # pack volts, for the tooltip + low-batt warning
        "ram":     _mem_percent(),
        "cpu":     _cpu_percent(),
        "uptime":  _uptime_str(),
        "speed":   round(_speed / 2.55),   # echo current speed % so the UI stays in sync
        "seed":    dict(_seed),            # {state, phase, planted, seeds}
    })


ui.on_message("motor_cmd",     logged("motor_cmd",     on_motor_cmd))
ui.on_message("motor_stop",    logged("motor_stop",    on_motor_stop))
ui.on_message("set_speed",     logged("set_speed",     on_set_speed))
ui.on_message("shutdown",      logged("shutdown",      on_shutdown))
ui.on_message("reboot",        logged("reboot",        on_reboot))
ui.on_message("connect_wifi",  logged("connect_wifi",  on_connect_wifi))
ui.on_message("start_hotspot", logged("start_hotspot", on_start_hotspot))
ui.on_message("seed_start",    logged("seed_start",    on_seed_start))
ui.on_message("seed_pause",    logged("seed_pause",    on_seed_pause))
ui.on_message("seed_stop",     logged("seed_stop",     on_seed_stop))
ui.on_message("plant_once",    logged("plant_once",    on_plant_once))
ui.on_message("soil_sample",   logged("soil_sample",   on_soil_sample))
ui.on_message("survey_start",  logged("survey_start",  on_survey_start))
ui.on_message("survey_stop",   logged("survey_stop",   on_survey_stop))
ui.on_message("set_camera",    logged("set_camera",    on_set_camera))
ui.on_message("drip_start",    logged("drip_start",    on_drip_start))
ui.on_message("drip_stop",     logged("drip_stop",     on_drip_stop))
ui.on_message("capture_start", logged("capture_start", on_capture_start))
ui.on_message("capture_stop",  logged("capture_stop",  on_capture_stop))
ui.on_message("capture_clear", logged("capture_clear", on_capture_clear))
ui.on_message("get_capture",   on_get_capture)
ui.on_message("get_soil",      on_get_soil)    # polled only while Soil tab is open
ui.on_message("get_vision",    on_get_vision)  # polled only while Camera tab is open
ui.on_message("get_stats",     on_get_stats)   # polled by the UI; unlogged (noisy)

App.run()
