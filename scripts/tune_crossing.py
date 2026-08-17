#!/usr/bin/env python3
"""Offline bench for detect_crossing — find the NEXT drip lateral, seen side-on.

WHY THIS EXISTS: a field run drove across two real laterals and recognised neither,
and the on-robot log could only say "found=False". Tuning a detector by editing a
constant, redeploying, driving the robot and reading one line per second is a terrible
loop. This one runs against saved frames in a second, so thresholds get chosen against
real pixels instead of guesses.

  # 1. COLLECT — Camera tab -> Dataset capture -> Start, then drive the robot slowly
  #    ACROSS a lateral so the tube sweeps down the frame. Stop. Pull the frames:
  scp "unoq:/home/arduino/ArduinoApps/motor-control/captures/*.jpg" crossing-frames/

  # 2. LOOK — what does the detector see in each frame?
  python scripts/tune_crossing.py crossing-frames/

  # 3. SWEEP — which thresholds would actually have worked?
  python scripts/tune_crossing.py crossing-frames/ --sweep

  # 4. SEE — write annotated copies to eyeball what it latched onto
  python scripts/tune_crossing.py crossing-frames/ --annotate out/

Run it on frames you KNOW contain a crossing tube. Its job is to tell you whether the
tube is detectable at all, and if so at what settings — not to confirm a hunch.
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "console", "python"))
from vision.vision import detect_crossing            # noqa: E402


def frames(path):
    if os.path.isfile(path):
        return [path]
    return sorted(os.path.join(path, f) for f in os.listdir(path)
                  if f.lower().endswith((".jpg", ".jpeg", ".png")))


def horizontal_lines(img, canny=(50, 150), tol=30, min_frac=0.30):
    """The raw Hough result, before any of detect_crossing's filtering.

    Separating this out answers the first question: is the tube producing a detectable
    LINE at all? If not, no threshold tuning will help and the problem is contrast,
    angle or resolution — not the darkness gate.
    """
    h, w = img.shape[:2]
    g = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    lines = cv2.HoughLinesP(cv2.Canny(g, *canny), 1, np.pi / 180, threshold=50,
                            minLineLength=int(w * min_frac), maxLineGap=30)
    out = []
    for x1, y1, x2, y2 in (np.asarray(lines).reshape(-1, 4) if lines is not None else []):
        ang = np.degrees(np.arctan2(y2 - y1, x2 - x1))
        if min(abs(ang), 180 - abs(ang)) < tol:
            out.append((x1, y1, x2, y2, ang))
    return out


def darkest_row(img):
    """The darkest horizontal band in the frame, and how dark it is.

    A crossing tube SHOULD be the darkest full-width band. If the darkest row is barely
    darker than the frame median, the tube is not distinguishable by brightness here and
    the darkness gate is the wrong discriminator for this soil.
    """
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    rows = g.mean(axis=1)
    y = int(np.argmin(rows))
    return y, float(rows[y]), float(np.median(g))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help="image file or folder of frames")
    ap.add_argument("--sweep", action="store_true", help="try threshold combinations")
    ap.add_argument("--annotate", metavar="OUTDIR", help="write annotated copies")
    a = ap.parse_args()

    fs = frames(a.path)
    if not fs:
        sys.exit("no images found in %s" % a.path)
    print("%d frame(s)\n" % len(fs))

    print("%-28s %-6s %-7s %-6s %-6s %s" % ("frame", "found", "tube_y", "near", "dark", "raw h-lines"))
    print("-" * 78)
    found = 0
    for f in fs:
        img = cv2.imread(f)
        if img is None:
            continue
        r = detect_crossing(img)
        found += bool(r["found"])
        print("%-28s %-6s %-7s %-6.2f %-6.2f %d" % (
            os.path.basename(f)[:28], r["found"], r["tube_y"], r["nearness"],
            r["dark_frac"], len(horizontal_lines(img))))

    print("\ndetected in %d of %d frames" % (found, len(fs)))

    # The two questions that decide what to do next
    img0 = cv2.imread(fs[len(fs) // 2])
    y, dark, med = darkest_row(img0)
    nlines = len(horizontal_lines(img0))
    print("\nmiddle frame: darkest row y=%d mean=%.0f, frame median=%.0f (contrast %.0f)"
          % (y, dark, med, med - dark))
    print("              near-horizontal Hough lines: %d" % nlines)
    if nlines == 0:
        print("  -> NO horizontal line is being found at all. Darkness thresholds are")
        print("     irrelevant; the tube is not producing an edge. Look at contrast,")
        print("     the camera angle, or minLineLength (tube may not span the frame).")
    elif med - dark < 15:
        print("  -> The darkest row is barely darker than the soil. Brightness is the")
        print("     WRONG discriminator here — the tube does not stand out.")

    if a.sweep:
        print("\n--- sweep: detections per setting (want high on tube frames) ---")
        print("%-12s %-12s %s" % ("dark_thresh", "min_dark_frac", "detected"))
        for dt in (90, 110, 125, 140, 160, 180):
            for mf in (0.05, 0.10, 0.22, 0.35):
                n = sum(bool(detect_crossing(cv2.imread(f), dark_thresh=dt,
                                             min_dark_frac=mf)["found"]) for f in fs)
                if n:
                    print("%-12d %-12.2f %d/%d" % (dt, mf, n, len(fs)))

    if a.annotate:
        os.makedirs(a.annotate, exist_ok=True)
        for f in fs:
            img = cv2.imread(f)
            if img is None:
                continue
            for x1, y1, x2, y2, _ in horizontal_lines(img):
                cv2.line(img, (x1, y1), (x2, y2), (0, 165, 255), 2)   # every h-line: orange
            r = detect_crossing(img)
            if r["tube_y"] is not None:
                c = (0, 255, 0) if r["found"] else (0, 0, 255)        # accepted green / rejected red
                cv2.line(img, (0, int(r["tube_y"])), (img.shape[1], int(r["tube_y"])), c, 2)
                cv2.putText(img, "dark=%.2f near=%.2f" % (r["dark_frac"], r["nearness"]),
                            (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, c, 1)
            cv2.imwrite(os.path.join(a.annotate, os.path.basename(f)), img)
        print("\nannotated frames -> %s  (orange = every horizontal line found," % a.annotate)
        print("                        green = accepted, red = rejected)")


if __name__ == "__main__":
    main()
