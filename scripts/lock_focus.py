#!/usr/bin/env python3
"""Find the sharpest focus for the QHM-999RL and LOCK it. Run with the camera MOUNTED and
aimed at the ground it will actually follow.

    ssh unoq 'bash -s' < scripts/lock_focus.sh        # the wrapper; this file is its payload

WHY THIS EXISTS. The QHM-999RL (SunplusIT 0806:0806) has CONTINUOUS AUTOFOCUS and powers up
with it enabled (focus_automatic_continuous=1, though the UVC default is 0). On a robot whose
camera sits at a fixed height above the ground, autofocus is pure liability:

  * Measured ON THE ROBOT at its real mounting height, sweeping focus_absolute 0..1023 at
    320x240: Laplacian-variance sharpness ran 28 to 324 — an 11.6x SPREAD with a clean unimodal
    peak at 272. (A room scene gave 58..1518, a 26x spread, peaking at 256 — same shape, more
    absolute detail. Focus is not a subtle effect on this lens at any distance.)
  * The tube detector gates on exactly what defocus destroys. _TUBE_MIN_FWHM_PX = 5 and
    _TUBE_MIN_SIGMA = 2.5 test the profile's width and contrast, so a hunting lens produces
    intermittent `fwhm-N<5` and `sigma-N<2.5` rejects with no cause visible in the frame.
  * Focus breathing also changes apparent magnification slightly, which quietly moves every
    pixel-to-metre constant while the run is in progress.

Autofocus is not stupid — on a room scene it settled at 265 against a measured optimum of 256 —
but it re-hunts whenever the scene changes, and each excursion costs up to that whole spread.
A fixed lens cannot hunt.

OPENCV CANNOT DO THIS. cap.set(CAP_PROP_AUTOFOCUS, ...) and cap.set(CAP_PROP_FOCUS, ...) both
return False on this camera and the properties read back 0.0 — verified on the board, where a
full OpenCV "sweep" produced sharpness 1172..1260 (scene noise) while the ioctl sweep on the
same camera produced 58..1518. So the OpenCV path silently does nothing, and a sweep written
against it measures noise and reports a confident wrong answer. This uses V4L2 ioctls directly.

WHY IT MUST BE RUN IN POSITION. Sharpness peaks at the distance the camera is actually looking
at. A value measured with the camera on a bench pointing across the room is optimal for the
room and wrong for ground at ~25cm. There is no way to guess it from the datasheet.
"""
import fcntl
import os
import struct
import sys
import time

import cv2

DEV = os.environ.get("CAM_DEV", "/dev/video0")
# Where the proof frame goes. In the wrapper's container only /probe is bind-mounted to the
# host, so writing to the container's own /tmp put the frame somewhere that vanished with
# --rm. A frame whose only purpose is to be looked at afterwards has to land outside.
OUT_DIR = os.environ.get("OUT_DIR", "/tmp")
# _IOWR('V', 28/27, struct v4l2_control { __u32 id; __s32 value; })  -- 8 bytes
VIDIOC_S_CTRL = 0xC008561C
VIDIOC_G_CTRL = 0xC008561B
CID_FOCUS_AUTO = 0x009A090C
CID_FOCUS_ABS = 0x009A090A


def _open_dev():
    try:
        return os.open(DEV, os.O_RDWR)
    except OSError as e:
        sys.exit("cannot open %s (%s). Is the camera plugged in, and is this the capture "
                 "node? /dev/video indices are NOT stable on this board." % (DEV, e))


def set_ctrl(fd, cid, val):
    try:
        fcntl.ioctl(fd, VIDIOC_S_CTRL, struct.pack("Ii", cid, val))
        return True
    except OSError:
        return False


def get_ctrl(fd, cid):
    try:
        buf = bytearray(struct.pack("Ii", cid, 0))
        fcntl.ioctl(fd, VIDIOC_G_CTRL, buf)
        return struct.unpack("Ii", bytes(buf))[1]
    except OSError:
        return None


def sharpness(cap, k=14):
    """Laplacian variance, median of k frames so exposure flicker cannot pick the winner."""
    vals = []
    for _ in range(k):
        ok, f = cap.read()
        if ok and f is not None:
            vals.append(cv2.Laplacian(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var())
    vals.sort()
    return vals[len(vals) // 2] if vals else 0.0


# NOTE: focus_auto and focus_absolute are set in SEPARATE ioctl calls, and must stay that way.
# focus_absolute is flagged INACTIVE while autofocus is on, so a combined VIDIOC_S_EXT_CTRLS
# (which is what `v4l2-ctl -c a=0 -c b=272` issues) fails atomically with EIO and applies
# NEITHER — leaving autofocus on while looking like it worked. See scripts/99-farmcam-focus.rules.
def measure(cap, fd, v, settle=0.7):
    set_ctrl(fd, CID_FOCUS_ABS, v)
    time.sleep(settle)                 # the lens is a motor; it needs real time to arrive
    for _ in range(6):
        cap.read()                     # flush frames captured at the OLD position
    return sharpness(cap)


def main():
    fd = _open_dev()
    if not set_ctrl(fd, CID_FOCUS_AUTO, 0):
        sys.exit("could not disable autofocus on %s" % DEV)
    if get_ctrl(fd, CID_FOCUS_AUTO) != 0:
        sys.exit("autofocus did not stay off — refusing to report a focus value")

    idx = int(DEV.rsplit("video", 1)[1])
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        sys.exit("OpenCV could not open index %d. If the console app is running it holds the "
                 "camera — stop it first (docker stop motor-control-main-1)." % idx)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    for _ in range(10):
        cap.read()

    print("coarse sweep (focus_absolute 0..1023):")
    coarse = []
    for v in range(0, 1024, 64):
        s = measure(cap, fd, v)
        coarse.append((s, v))
        print("  %4d : %8.1f" % (v, s))
    coarse.sort(reverse=True)
    peak = coarse[0][1]

    lo, hi = max(0, peak - 64), min(1023, peak + 64)
    print()
    print("fine sweep %d..%d:" % (lo, hi))
    fine = []
    for v in range(lo, hi + 1, 16):
        s = measure(cap, fd, v)
        fine.append((s, v))
        print("  %4d : %8.1f" % (v, s))
    fine.sort(reverse=True)
    best_s, best_v = fine[0]
    worst_s = coarse[-1][0]

    # Centre of the plateau, not the single highest sample: neighbouring values are often within
    # noise of each other, and sitting mid-plateau means thermal or mechanical drift in either
    # direction costs the least sharpness.
    near = sorted(v for s, v in fine if s >= 0.98 * best_s)
    chosen = near[len(near) // 2] if near else best_v

    print()
    print("sharpest sample : focus=%d (%.1f)" % (best_v, best_s))
    print("plateau         : %s" % (", ".join(str(v) for v in near) or "single point"))
    print("CHOSEN          : focus=%d  (plateau centre)" % chosen)
    print("worst in sweep  : %.1f  -> best/worst = %.1fx" % (
        worst_s, best_s / worst_s if worst_s else 0.0))
    if best_s < 1.4 * worst_s:
        print()
        print("WARNING: sharpness barely varied across the whole range. Either the scene has no")
        print("         detail to focus on, or the lens is not moving. Do NOT trust this value.")

    set_ctrl(fd, CID_FOCUS_ABS, chosen)
    time.sleep(0.8)
    for _ in range(6):
        cap.read()
    ok, f = cap.read()
    if ok and f is not None:
        out = os.path.join(OUT_DIR, "focus_locked_%d.jpg" % chosen)
        cv2.imwrite(out, f)
        print()
        print("saved %s — LOOK AT IT before trusting the number." % out)
    print()
    print("Focus is now locked at %d for THIS SESSION only — these are driver-side controls"
          % chosen)
    print("with no persistence, and the camera powers up with autofocus ON.")
    print("To make it survive replug and reboot, put this value in")
    print("scripts/99-farmcam-focus.rules and install it (instructions at the top of that file).")
    cap.release()
    os.close(fd)


if __name__ == "__main__":
    main()
