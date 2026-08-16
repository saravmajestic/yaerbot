"""
Farm OS — Stage 5 tube-following control loop (runs ON the UNO Q).

Vision-based autonomous navigation: read ESP32-CAM frame -> detect_tube ->
steer via setMotors (RouterBridge) -> at a confirmed emitter, stop and plant.
This is the AI showcase — CV on the Qualcomm side, real-time control on the STM32.

NOTE: imports arduino.app_utils, so it only runs on-device (like the other
test_*.py scripts). The pure-CV bits (vision.py) are unit-tested off-device.

  python3 tube_follow.py http://<esp32-ip>:81/stream
"""
import sys
import time
import json

from arduino.app_utils import Bridge, App          # on-device only
from vision.camera import FrameSource
from vision.vision import detect_tube, detect_emitter

# ── tunables ────────────────────────────────────────────────────────────────
BASE_PWM      = 77      # ~30% of 255 while following (plan.md)
STEER_GAIN    = 10      # PWM per unit of correction (-5..+5)
EMIT_CONF     = 0.55    # confidence to accept an emitter
LOST_STOP_S   = 1.0     # stop if the tube is lost this long
EMIT_COOLDOWN = 3.0     # don't re-fire on the same emitter within this window
LEFT_TRIM     = 0.77    # match main.py motor trim (goes straight)
RIGHT_TRIM    = 1.00


def _drive(correction):
    """Differential drive: steer toward the tube. +correction = tube right -> turn right."""
    left  = BASE_PWM + STEER_GAIN * correction
    right = BASE_PWM - STEER_GAIN * correction
    left  = int(max(0, min(255, left  * LEFT_TRIM)))
    right = int(max(0, min(255, right * RIGHT_TRIM)))
    Bridge.call("setMotors", left, right)


def _moisture():
    try:
        m = json.loads(Bridge.call("getMoisture"))
        return min(m.get("a0", 16383), m.get("a1", 16383))   # wetter (lower) of the two
    except Exception:
        return None


def run(source):
    cam = FrameSource(source)
    print("tube_follow: source=%r" % source, flush=True)
    last_seen = time.time()
    last_plant = 0.0

    while True:
        ok, frame = cam.read()
        if not ok:
            Bridge.call("stop"); time.sleep(0.1); continue

        tube = detect_tube(frame)

        # emitter check (visual + moisture confirm), with a cooldown
        emit = detect_emitter(frame, moisture=_moisture())
        if emit["detected"] and emit["confidence"] >= EMIT_CONF \
                and (time.time() - last_plant) > EMIT_COOLDOWN:
            print("EMITTER confirmed (conf=%.2f) -> stop + plant" % emit["confidence"], flush=True)
            Bridge.call("stop")
            Bridge.call("plantSeed")
            last_plant = time.time()
            continue

        if tube["found"]:
            last_seen = time.time()
            _drive(tube["correction"])
        elif time.time() - last_seen > LOST_STOP_S:
            Bridge.call("stop")           # lost the line -> hold

        time.sleep(0.05)                  # ~20 Hz control loop


if __name__ == "__main__":
    src = sys.argv[1] if len(sys.argv) > 1 else "http://192.168.4.50:81/stream"
    App.run(user_loop=lambda: run(src))
