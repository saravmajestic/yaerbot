#!/usr/bin/env python3
"""Calibrate PX_PER_M_DEPTH by pushing the robot a measured distance BY HAND. No motors.

    scp scripts/calib_odometer.py unoq:/tmp/
    ssh unoq 'bash -s' < scripts/lock_focus.sh -- --odo     # (or run the docker line below)

WHY THIS IS NEEDED. PX_PER_M_DEPTH = 1090.0 was measured with the Logitech C310. The robot now
has a QHM-999RL, a different lens with a different field of view, so the constant is a C310
number applied to a camera that has never been measured. It is not cosmetic: the odometer's
whole output is

    distance_m = |net_dy| / px_per_m

so an error here is a straight multiplicative error on every distance the camera reports.

WHY THE 43cm FRAME MEASUREMENT DOES NOT SETTLE IT. The frame covers 43cm of ground top to
bottom, which naively gives 240px / 0.43m = 558 px/m. But FlowOdometer uses roi=(0.5, 1.0) —
the LOWER HALF of the frame only — and the view is oblique, so the bottom half covers
considerably less than half of that 43cm. The scale in the ROI is therefore higher than 558,
by an amount that depends on the camera's tilt and height. Deriving it needs geometry we have
not measured; measuring it end to end needs a tape measure and one push. Hence this.

  For reference: 1090 px/m would mean the lower half of the frame covers 120/1090 = 11cm of
  ground, out of the 43cm the whole frame covers. 558 would mean 21.5cm — i.e. no perspective
  compression at all, which is certainly wrong. The true value is between, and the point of
  this script is to stop guessing which.

HOW TO USE
  1. Put the robot on the ground it will actually run on, with a tape measure alongside.
  2. Start this. It prints the accumulated distance continuously.
  3. Push the robot SLOWLY and STRAIGHT along exactly 1.000 m, then stop.
  4. Read the final number and apply:

         PX_PER_M_DEPTH_new = 1090.0 * (reported_m / real_m)

     Under-reporting (reported < real) means the old constant was too HIGH.
  5. Put the result in vision/odometry.py PX_PER_M_DEPTH *and* main.py _PX_PER_M_DEPTH — they
     are two separate literals and both must move together.

Push SLOWLY. The odometer rejects any frame-to-frame shift above MAX_SHIFT_PX (60px); pushing
fast enough to exceed that silently drops frames and under-reports. The "rejected" count in the
output is how you tell — if it climbs while you push, you are pushing too fast.
"""
import os
import sys
import time

import cv2

sys.path.insert(0, "/app/python")

from vision.odometry import FlowOdometer, PX_PER_M_DEPTH   # noqa: E402

DEV_INDEX = int(os.environ.get("CAM_INDEX", "0"))
REAL_M = float(os.environ.get("REAL_M", "1.0"))


def main():
    cap = cv2.VideoCapture(DEV_INDEX)
    if not cap.isOpened():
        sys.exit("could not open camera index %d — is the console app still holding it? "
                 "(docker stop motor-control-main-1)" % DEV_INDEX)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    for _ in range(12):
        cap.read()

    odo = FlowOdometer()
    print("PX_PER_M_DEPTH currently %.1f — measuring against a real %.3f m" % (
        PX_PER_M_DEPTH, REAL_M))
    print("Push the robot SLOWLY and STRAIGHT. Ctrl-C when you reach the mark.")
    print()
    t0 = time.time()
    try:
        while True:
            ok, f = cap.read()
            if not ok or f is None:
                continue
            odo.update(f)
            s = odo.stats()
            if time.time() - t0 > 0.5:
                t0 = time.time()
                sys.stdout.write("\r  distance %.4f m | frames %d | rejected %d   " % (
                    s["distance_m"], s["updates"], s["rejected"]))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass

    s = odo.stats()
    rep = s["distance_m"]
    print()
    print()
    print("reported : %.4f m over a real %.3f m" % (rep, REAL_M))
    print("frames   : %d used, %d rejected%s" % (
        s["updates"], s["rejected"],
        "  <-- high: were you pushing too fast? (MAX_SHIFT_PX=60)"
        if s["rejected"] > 0.2 * max(s["updates"], 1) else ""))
    if rep <= 0.01:
        print()
        print("NO MOTION MEASURED. Either nothing moved, or every frame was rejected. Do not")
        print("derive a constant from this.")
        return
    new = PX_PER_M_DEPTH * (rep / REAL_M)
    print()
    print("PX_PER_M_DEPTH should be %.1f  (was %.1f, a factor of %.2fx)" % (
        new, PX_PER_M_DEPTH, new / PX_PER_M_DEPTH))
    print()
    print("Implied geometry check: the lower half of the frame then covers %.1f cm of ground."
          % (100.0 * 120.0 / new))
    print("The whole frame covers 43 cm, so that half must be under 21.5 cm and, because the")
    print("view is oblique, meaningfully under it. If the number above is outside roughly")
    print("8-20 cm, distrust the run and repeat it.")
    print()
    print("Put it in BOTH vision/odometry.py (PX_PER_M_DEPTH) and main.py (_PX_PER_M_DEPTH).")
    cap.release()


if __name__ == "__main__":
    main()
