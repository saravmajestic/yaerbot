"""THE OTHER HALF: real frames with NO followable tube, which must come back not-found.

Every frame set used to tune this detector — 2086 dense captures, the 303 set, the golden seven —
contains a tube. So every measurement made all day answered "does it find the tube when there is
one?" and NOTHING answered "does it stay quiet when there isn't?". That asymmetry is what let
_MIN_BANDS move four times in one day: each move improved recall on positives, and the cost was
invisible because no negative was ever scored.

These nine are real field frames from a crossing pass: bare soil with a lateral running ACROSS
the direction of travel. They are the hardest useful negatives, because they contain a strong,
high-contrast, genuinely tube-shaped feature at the wrong orientation — exactly what fools a
detector that looks for "a dark line" rather than "the line we are driving along". Two are also
partly corrupt (a truncated MJPEG frame decodes with a solid green block), which is a second
negative class worth holding onto: the camera does produce these.

WHAT THIS MEASURED, first time it was ever run:
  cold   (no hint):  0 of 9 false positives
  hinted (hint=160): 4 of 9 false positives from detect_tube, of which the console's fwhm and
                     sigma gates reject 3, leaving 1 that passes every gate and reaches the
                     steering code with correction = -0.98, i.e. near full-scale.

The hinted column is the one that matters, because hinted IS the tracker's steady state. So the
seeded search — committed the same day — measurably trades false negatives for false positives,
and this file is where that trade is kept honest rather than assumed.
"""
import os
import sys

import cv2
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "console", "python"))

import vision.vision as V                                    # noqa: E402

FRAMES = os.path.join(_HERE, "frames", "negative")

# All nine contain a crossing lateral over bare soil and NO tube along the travel direction, so
# the correct answer for every one of them is "not found".
NEGATIVES = sorted(f for f in os.listdir(FRAMES) if f.lower().endswith((".jpg", ".png")))

# KNOWN FALSE POSITIVE, recorded rather than hidden. This frame is partly corrupt AND its
# crossing lateral runs diagonally, so its three bands march steadily left (174.5 -> 157.5 ->
# 144.0) and read as a tube leaning at -14.3 deg — inside the 35 deg angle gate — with a real
# 14px width that passes both width gates. Nothing currently distinguishes it from a genuine
# leaning tube using one frame alone; the fix is temporal (a crossing is transient, the followed
# row is not), which is the traverse latch's job and is not implemented for this case.
# Left as xfail so the hole stays visible and closes itself if the detector improves.
KNOWN_FALSE_POSITIVE = {"cap_20260817_015415_692.jpg"}


def _load(name):
    p = os.path.join(FRAMES, name)
    assert os.path.exists(p), "negative frame missing from the repo: %s" % p
    im = cv2.imread(p)
    assert im is not None, "could not decode %s" % p
    return im


def test_there_are_negatives_at_all():
    """Guards the whole file: if the fixtures vanish, NEGATIVES goes empty and every
    parametrised test below silently disappears into a green run."""
    assert len(NEGATIVES) >= 9, "expected >=9 negative frames, found %d" % len(NEGATIVES)


@pytest.mark.parametrize("name", NEGATIVES)
def test_no_tube_is_found_cold(name):
    """No hint — first frame of a run, or after a genuine loss. Must find nothing."""
    t = V.detect_tube(_load(name))
    assert not t["found"], "%s: FALSE POSITIVE cold at x=%.0f (no tube in this frame)" % (
        name, t["x_near"])


@pytest.mark.parametrize("name", NEGATIVES)
def test_no_tube_survives_the_console_gates_when_hinted(name, request):
    """WITH a hint, then through the console's own reject gates — the real steady-state path.

    detect_tube alone is not the product: main.py's _tube_reject drops readings that are found
    but not tube-shaped. What must hold is that nothing reaches the steering code, so this
    asserts on the COMBINATION, which is what actually drives the robot.
    """
    if name in KNOWN_FALSE_POSITIVE:
        request.node.add_marker(pytest.mark.xfail(
            reason="diagonal crossing lateral reads as a leaning tube; needs a temporal fix",
            strict=True))
    sys.path.insert(0, _HERE)
    from test_console_imports import _load_main
    main = _load_main()

    t = V.detect_tube(_load(name), hint_x=160)
    if not t["found"]:
        return                                   # detector rejected it outright, done
    why = main._tube_reject(t)
    assert why is not None, \
        "%s: passed EVERY gate and would steer — x=%.0f correction=%+.2f fwhm=%s width=%s" % (
            name, t["x_near"], t["correction"], t.get("width_fwhm"), t.get("width"))


def test_hinting_does_not_make_false_positives_worse_than_this():
    """A RATCHET on the cost of seeding. Hinting bought better recall on positives and paid for
    it in false positives; this pins the price at what was measured (1 of 9 through the gates) so
    a future recall tweak cannot quietly raise it.
    """
    sys.path.insert(0, _HERE)
    from test_console_imports import _load_main
    main = _load_main()

    through = []
    for name in NEGATIVES:
        t = V.detect_tube(_load(name), hint_x=160)
        if t["found"] and main._tube_reject(t) is None:
            through.append("%s@%.0f" % (name, t["x_near"]))
    assert len(through) <= 1, \
        "false positives through the gates rose to %d (was 1): %s" % (
            len(through), ", ".join(through))
