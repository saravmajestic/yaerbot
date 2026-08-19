"""SUITE 3 (part 2): finding the NEXT lateral while traversing, on real frames.

detect_crossing() is a different detector from detect_tube(): it looks for a near-HORIZONTAL band
(a lateral running across the path) and reports how far down the frame it sits, as `tube_y` and a
normalised `nearness`. The traverse phase drives sideways until one is close enough to turn onto.

WHY THESE FRAMES. tests/frames/negative/ was captured as a crossing pass and is used elsewhere as
NEGATIVES for detect_tube — bare soil with a lateral running the wrong way. The same frames are
POSITIVES here, which is a useful property: one set, two opposite requirements, and a detector that
cheated on either would fail the other.

Measured on them, in capture order:

    015250  found  y=23.0   near=0.10  w=36  s=6.1 dark
    015253  found  y=23.5   near=0.10  w=38  s=6.1 dark
    015314  found  y=23.5   near=0.10  w=38  s=6.1 dark
    015329  found  y=78.0   near=0.33  w=48  s=3.2 bright
    015356  found  y=78.5   near=0.33  w=46  s=3.2 bright
    015359  found  y=78.0   near=0.33  w=48  s=3.2 bright
    015403  NOT found
    015415  found  y=171.5  near=0.71  w=26  s=3.1 dark
    015431  NOT found                              <- truncated MJPEG frame, half solid green

That is a real approach: the band sweeps DOWN the frame as the robot closes on it, which is exactly
the signal _traverse_track gates on (a shadow or straw does not advance with distance). And
lat2_found_023307, the frame the robot actually turned on in the field, reproduces its logged values
exactly: y=212.0, nearness 0.88.

NOT COVERED HERE: the approach-gate arithmetic itself (sight fraction, grew >= need_px) already has
unit coverage in test_gates.py. This file is about whether the DETECTOR sees real laterals in real
frames, which no amount of gate testing can answer.
"""
import os
import sys

import cv2
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "console", "python"))

from vision.vision import detect_crossing              # noqa: E402
from test_console_imports import _load_main            # noqa: E402

NEG = os.path.join(_HERE, "frames", "negative")
ALIGN = os.path.join(_HERE, "frames", "align")

# The crossing pass, in capture order. (filename, expect_found, approx nearness or None)
APPROACH = [
    ("cap_20260817_015250_845.jpg", True,  0.10),
    ("cap_20260817_015253_217.jpg", True,  0.10),
    ("cap_20260817_015314_057.jpg", True,  0.10),
    ("cap_20260817_015329_456.jpg", True,  0.33),
    ("cap_20260817_015356_218.jpg", True,  0.33),
    ("cap_20260817_015359_741.jpg", True,  0.33),
    ("cap_20260817_015403_580.jpg", False, None),
    ("cap_20260817_015415_692.jpg", True,  0.71),
    ("cap_20260817_015431_098.jpg", False, None),   # truncated frame, half green
]


def _load(d, name):
    p = os.path.join(d, name)
    assert os.path.exists(p), "fixture missing: %s" % p
    im = cv2.imread(p)
    assert im is not None, "could not decode %s" % p
    return im


# ---------------------------------------------------------------- the detector sees real laterals

@pytest.mark.parametrize("name,expect,near", APPROACH)
def test_crossing_detected_where_expected(name, expect, near):
    c = detect_crossing(_load(NEG, name))
    assert c["found"] == expect, "%s: found=%s, expected %s" % (name, c["found"], expect)
    if expect:
        assert abs(c["nearness"] - near) <= 0.12, "%s: nearness %.2f, expected ~%.2f" % (
            name, c["nearness"], near)


def test_the_band_sweeps_DOWN_the_frame_as_the_robot_closes():
    """The whole basis of the approach gate: a real lateral advances with distance driven, a shadow
    does not. If this ordering breaks, _traverse_track's `grew` term is measuring nothing."""
    ys = [detect_crossing(_load(NEG, n))["tube_y"]
          for n, e, _ in APPROACH if e]
    # NOT strict monotonicity: consecutive frames of the same scene differ by sub-pixel noise, and
    # the first version of this test failed on 78.5 -> 78.0. What matters is that the band ADVANCES
    # and never goes meaningfully backwards, so allow a small backward tolerance.
    JITTER_PX = 2.0
    worst_back = min([b - a for a, b in zip(ys, ys[1:])] or [0.0])
    assert worst_back >= -JITTER_PX, \
        "the crossing moved backwards by %.1f px (>%.0f noise) during the approach: %s" % (
            -worst_back, JITTER_PX, ys)
    assert ys[-1] - ys[0] > 100, "expected a large sweep across the pass, got %.0f px" % (
        ys[-1] - ys[0])


def test_the_decision_frame_reproduces_its_field_numbers():
    """lat2_found_023307 is the frame the robot turned on during the 02:33 run, which logged
    'nearness 0.88, y=212.0, w=80, 3.5 sigma dark'. If this drifts, every conclusion drawn from that
    run is void."""
    c = detect_crossing(_load(ALIGN, "lat2_found_023307_576.jpg"))
    assert c["found"]
    assert abs(c["tube_y"] - 212.0) <= 3, "y=%.1f, field logged 212.0" % c["tube_y"]
    assert abs(c["nearness"] - 0.88) <= 0.03, "nearness %.2f, field logged 0.88" % c["nearness"]
    assert c["polarity"] == "dark"


def test_a_truncated_frame_does_not_crash_the_detector():
    """015431 is a real MJPEG frame the camera mangled — the bottom is solid green. The camera does
    produce these, and a detector that raises on one takes the whole camera loop down."""
    c = detect_crossing(_load(NEG, "cap_20260817_015431_098.jpg"))
    assert c["found"] is False


# ---------------------------------------------------------------- arrival

def test_arrival_threshold_is_reached_only_by_the_closest_frame():
    """_ARRIVE_NEAR gates the turn. Firing early is what left the lateral beside the robot on
    2026-08-17 (turn at nearness 0.61, then no vertical tube at all in the next frame)."""
    mod = _load_main()
    reached = []
    for n, e, _ in APPROACH:
        if not e:
            continue
        c = detect_crossing(_load(NEG, n))
        if c["nearness"] >= mod._ARRIVE_NEAR:
            reached.append((n, c["nearness"]))
    assert not reached, ("frames from mid-approach reached the arrival threshold %.2f: %s"
                        % (mod._ARRIVE_NEAR, reached))

    c = detect_crossing(_load(ALIGN, "lat2_found_023307_576.jpg"))
    assert c["nearness"] >= mod._ARRIVE_NEAR - 0.01, (
        "the frame the robot actually turned on (%.2f) is below the arrival gate %.2f"
        % (c["nearness"], mod._ARRIVE_NEAR))


# ---------------------------------------------------------------- barebones, to fill in

@pytest.mark.skip(reason="TODO: needs a multi-lateral capture. Replay a full traverse — end of row, "
                         "turn, drive across, find the next lateral, turn on — and assert the "
                         "lateral count and the distance traversed between them. No such capture "
                         "exists yet; the 2026-08-19 runs were single-lateral.")
def test_full_traverse_finds_every_lateral_in_a_multi_lateral_capture():
    """The end-to-end case: N laterals in, N laterals found, gaps matching the row spacing."""


@pytest.mark.skip(reason="TODO: needs frames of bare soil captured while TRAVERSING (no lateral "
                         "anywhere in view). The negative set all contain a crossing, so there is "
                         "nothing here to measure the crossing detector's false-positive rate "
                         "against — and it does false-positive: 2 of 4 golden following frames "
                         "report a crossing, one at nearness 0.69.")
def test_no_crossing_is_reported_on_bare_traverse_ground():
    """The false-positive half. Capture: drive the traverse leg over ground with no lateral."""
