#!/usr/bin/env python3
"""Compare grayscale against illumination-invariant channels for tube detection under shadow.

    python3 scripts/shadow_channel_probe.py            # runs on the committed fixture sets

WHY. The tube detector profiles a GRAYSCALE image, and grayscale cannot separate a dark tube from a
dark shadow. Measured on tests/frames/shadow/cap_20260818_083851_243.jpg, which has a hard horizontal
shadow edge crossing a strong vertical tube:

    region             V       S      R-B
    tube, lit        80.9    73.9   -23.4
    tube, shadow     36.6   145.4   -20.8
    soil, lit       172.1    10.0    +5.7
    soil, shadow     42.0    45.8    -6.1

BRIGHTNESS DOES NOT SEPARATE THEM: tube-in-shadow is V=36.6 against shadowed soil at V=42.0. That is
the failure, in one number.

R-B IS NEARLY INVARIANT TO THE ILLUMINATION: the tube reads -23.4 lit and -20.8 shadowed — it barely
moves — while soil ranges -6 to +6. This is what the literature predicts: a shadow changes the
COLOUR of the illumination, not merely its intensity, so a ratio or difference of channels is
approximately preserved across a shadow boundary on one material, and differs between materials
regardless of how dark either is. Finlayson et al., "On the Removal of Shadows From Images"
(https://www.cs.sfu.ca/~mark/ftp/Pami06/pami06.pdf) develops the principled 1D log-chromaticity
projection; the channel differences here are the cheap approximation to it.

NOTE the tube is NOT achromatic — S = 74-145. An earlier guess that black plastic would be neutral
and soil coloured is backwards: LIT SOIL is the achromatic one (S = 10). Saturation is therefore a
poor discriminator and the channel DIFFERENCE is the useful signal.

WHAT THIS MEASURED, hinted (the follow loop's steady state), on the committed fixtures:

    golden set (7 frames, labelled)      gray 5/7   R-B 5/7   log R/B 5/7   R-G 5/7
    but they fail on DIFFERENT frames:
      gray misses  cap_20260818_105926_367 (harsh midday, a documented xfail)
                   -> log R/B finds it at x=182 against a truth of 180
      invariants miss live_1245 -> gray finds it

    shadow set (6 frames, UNLABELLED)
      gray finds nothing at all on 022932 and 083851 — both plainly have a tube
      R-B finds both, at x=160 and x=188, matching a by-eye reading of ~150 and ~185

CONCLUSION: not a replacement, a SECOND OPINION. Running the profile on grayscale and on a
log-chromaticity channel and preferring the better-supported fit should pick up the shadow cases
without losing the ones grayscale already handles. detect_tube costs ~6 ms against a ~43 ms frame
budget, so a second channel pass is affordable.

NOT YET VALIDATED: the shadow set has no ground-truth labels, so its "wins" are judged against a
by-eye reading good to about +/-20 px. Label those six frames before acting on this.
"""
import os
import sys

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "console", "python"))

import vision.vision as V                                    # noqa: E402

FRAMES = os.path.join(os.path.dirname(_HERE), "tests", "frames")

# Golden labels, kept in step with tests/test_golden_frames.py
GOLDEN = {"lost_121524.jpg": 168, "cap_20260818_110504_150.jpg": 175,
          "cap_20260818_110502_124.jpg": 178, "emit1_lat1_111611.jpg": 178,
          "cap_20260818_105926_367.jpg": 180, "cap_20260818_104428_437.jpg": 55,
          "live_1245.png": 150}

MODES = ("gray", "rb", "logrb", "rg")


def as_channel(im, mode):
    """Return a 3-channel 8-bit image whose grayscale IS the requested channel.

    The tube is strongly negative in every difference channel and soil is near zero, so the tube
    stays DARK — the same polarity grayscale gives it, which means detect_tube needs no changes to
    read these.
    """
    if mode == "gray":
        return im
    b, g, r = [im[:, :, i].astype(np.float32) for i in range(3)]
    if mode == "rb":
        d = r - b
    elif mode == "logrb":
        d = (np.log(r + 1.0) - np.log(b + 1.0)) * 60.0
    elif mode == "rg":
        d = r - g
    else:
        raise ValueError(mode)
    d = np.clip((d + 60.0) * (255.0 / 120.0), 0, 255).astype(np.uint8)
    return cv2.cvtColor(d, cv2.COLOR_GRAY2BGR)


def main():
    print("%-34s %5s %s" % ("frame", "truth", "".join("%10s" % m for m in MODES)))
    score = dict.fromkeys(MODES, 0)
    for f, truth in sorted(GOLDEN.items()):
        im = cv2.imread(os.path.join(FRAMES, "golden", f))
        if im is None:
            continue
        cells = []
        for m in MODES:
            t = V.detect_tube(as_channel(im, m), hint_x=truth)
            if t["found"] and abs(t["x_near"] - truth) <= 50:
                score[m] += 1
                cells.append("%.0f ok" % t["x_near"])
            else:
                cells.append("%.0f" % t["x_near"] if t["found"] else "none")
        print("%-34s %5d %s" % (f[:34], truth, "".join("%10s" % c for c in cells)))
    print("\ngolden, within 50px hinted: %s" % score)

    sd = os.path.join(FRAMES, "shadow")
    if os.path.isdir(sd):
        print("\nshadow set (UNLABELLED — positions are for eyeballing, not scoring)")
        print("%-34s %s" % ("frame", "".join("%10s" % m for m in MODES)))
        for f in sorted(os.listdir(sd)):
            im = cv2.imread(os.path.join(sd, f))
            if im is None:
                continue
            cells = []
            for m in MODES:
                t = V.detect_tube(as_channel(im, m))
                cells.append("%.0f" % t["x_near"] if t["found"] else "none")
            print("%-34s %s" % (f[:34], "".join("%10s" % c for c in cells)))


if __name__ == "__main__":
    main()
