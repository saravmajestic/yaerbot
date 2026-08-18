#!/usr/bin/env python3
"""Score the deployed FOMO emitter model against a folder of frames. RUNS ON THE BOARD.

WHY THIS EXISTS. On 2026-08-18 the model was scored against the 77 USB frames and fired on
56 of them with a MEDIAN confidence of 0.916 — including 0.997 on a frame that a human
inspection confirmed held nothing but plain drip tube. The distribution ran 0.603 to 0.997
with no gap in it, which is the thing worth measuring: when the scores on tube and the scores
on emitters overlap completely, NO value of _EMIT_CONF separates them and threshold tuning is
a waste of a day. That is only visible if you score a whole folder instead of eyeballing the
live overlay.

So: run this BEFORE and AFTER any retrain, and compare. The bar to beat is recorded in
docs/ml-emitter-model.md.

    # on the board, inside the app container (the brick talks to the runner over HTTP)
    docker cp captures/usb motor-control-main-1:/tmp/usb
    docker exec motor-control-main-1 python3 /app/scripts/score_emitter_model.py /tmp/usb

Optionally pass a file of ground truth ("<filename> emitter" or "<filename> none" per line)
to get precision/recall instead of just a distribution.
"""
import glob
import os
import sys

import cv2

sys.path.insert(0, "/app/python")
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "console", "python"))

from vision.emitter_ml import detect_emitter_ml, ml_available   # noqa: E402

# Deliberately far below _EMIT_CONF. The question is not "what passes our gate" but "what does
# the model propose at all" — a model that proposes a box on every frame is broken no matter
# where the gate sits, and raising the gate hides exactly that.
PROBE_CONF = 0.05


def _truth(path):
    """{basename: True/False} from a ground-truth file, or {} if none given."""
    if not path:
        return {}
    out = {}
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) >= 2:
                out[os.path.basename(parts[0])] = parts[1].strip().lower() in (
                    "emitter", "1", "yes", "true")
    return out


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    folder = sys.argv[1]
    truth = _truth(sys.argv[2] if len(sys.argv) > 2 else None)

    if not ml_available():
        print("MODEL NOT AVAILABLE — the brick could not reach a deployed model.")
        print("Check:  curl -s http://127.0.0.1:1337/api/info")
        return 1

    files = sorted(glob.glob(os.path.join(folder, "*.jpg")) +
                   glob.glob(os.path.join(folder, "*.png")))
    if not files:
        print("no frames in %s" % folder)
        return 1

    rows = []
    for f in files:
        im = cv2.imread(f)
        if im is None:
            continue
        r = detect_emitter_ml(im, moisture=None, conf_min=PROBE_CONF)
        rows.append((os.path.basename(f), r.get("ml_value"), r.get("position")))

    hits = [(n, v, p) for n, v, p in rows if v is not None]
    print("frames scored          : %d" % len(rows))
    print("frames with ANY box    : %d  (%.0f%%)  at conf >= %.2f"
          % (len(hits), 100.0 * len(hits) / len(rows), PROBE_CONF))
    if hits:
        vals = sorted(v for _, v, _ in hits)
        print("confidence             : min %.3f  median %.3f  max %.3f"
              % (vals[0], vals[len(vals) // 2], vals[-1]))
        for gate in (0.5, 0.7, 0.9):
            print("  frames clearing %.2f  : %d" % (gate, sum(1 for v in vals if v >= gate)))

    if truth:
        # Precision/recall at our real gate. THIS is the number that decides a retrain worked.
        gate = 0.90
        tp = fp = fn = tn = 0
        unknown = 0
        for n, v, _p in rows:
            if n not in truth:
                unknown += 1
                continue
            fired = v is not None and v >= gate
            if truth[n] and fired:
                tp += 1
            elif truth[n]:
                fn += 1
            elif fired:
                fp += 1
            else:
                tn += 1
        print()
        print("against ground truth at gate %.2f:" % gate)
        print("  true positive %d | false positive %d | false negative %d | true negative %d"
              % (tp, fp, fn, tn))
        if tp + fp:
            print("  precision %.2f" % (tp / float(tp + fp)))
        if tp + fn:
            print("  recall    %.2f" % (tp / float(tp + fn)))
        if unknown:
            print("  (%d frames had no ground-truth entry)" % unknown)
        print()
        print("A retrain has WORKED when the plain-tube frames stop firing — i.e. false")
        print("positives go to near zero while recall holds. It has NOT worked if the only")
        print("change is that everything scores lower; that just moves the overlap.")
    else:
        print()
        print("No ground truth given, so this is a distribution only. The tell-tale of the")
        print("broken model is a HIGH median with a HIGH fire-rate: it means the model is")
        print("answering 'tube', not 'emitter'. Pass a ground-truth file for precision/recall.")

    # The worst offenders are the useful output: these are the frames to open and look at.
    if hits:
        print()
        print("highest-confidence frames (open these and check what is actually there):")
        for n, v, p in sorted(hits, key=lambda x: -x[1])[:12]:
            mark = ""
            if truth and n in truth:
                mark = "  [truth: %s]" % ("emitter" if truth[n] else "NO emitter")
            print("  %-34s %.3f  at %s%s" % (n, v, p, mark))
    return 0


if __name__ == "__main__":
    sys.exit(main())
