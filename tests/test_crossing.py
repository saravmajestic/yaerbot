"""detect_crossing — finding the NEXT drip lateral while driving ACROSS the rows.

Between laterals the robot searches for the next tube rather than computing where it
should be, so the rows need not be parallel or evenly spaced. That means the detector
sees the tube side-on, as a near-HORIZONTAL line — which detect_tube deliberately
throws away, because rejecting crosswise lines is exactly what makes tube-following
immune to furrows and shadows.

The risk this inverts: crossing a field, furrows and tyre tracks run horizontally too.
These tests pin the two discriminators that separate a tube from a furrow.
"""
import os
import sys

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2", reason="OpenCV not installed off-device")

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "console", "python"))

from vision.vision import detect_crossing, detect_tube


def soil(seed=0):
    rng = np.random.default_rng(seed)
    f = np.full((240, 320, 3), 150, np.uint8)
    return (f + rng.integers(-12, 12, f.shape)).astype(np.uint8)


def with_line(kind, y=150, shade=35, seed=0):
    f = soil(seed)
    if kind == "horizontal":
        cv2.line(f, (0, y), (319, y + 2), (shade,) * 3, 9)
    else:                                   # vertical
        cv2.line(f, (160, 0), (162, 239), (shade,) * 3, 9)
    return f


def test_finds_a_dark_crossing_tube():
    r = detect_crossing(with_line("horizontal"))
    assert r["found"]
    assert 140 < r["tube_y"] < 165


def test_finds_a_BRIGHT_crossing_tube_too():
    """CHANGED 2026-08-17, and the change is the point. This test used to assert the
    opposite — that a bright horizontal line must be REJECTED, on the theory that only
    a sunlit furrow crest would be bright. Measurement on real frames from one run
    killed that theory: following the row the tube read 170 mean against soil at 157
    (brighter), and ninety seconds later, crossing, 112 against 176 (darker). Same tube.
    On a serpentine path it flips every row, because alternate rows face into and away
    from the sun. A detector that only accepts dark tubes is blind half the time."""
    r = detect_crossing(with_line("horizontal", shade=205))
    assert r["found"] and r["polarity"] == "bright"


def test_reports_which_way_the_contrast_went():
    """Polarity is measured, never assumed — and it is logged, so a field run says
    which lighting regime it was in rather than leaving it to be inferred."""
    assert detect_crossing(with_line("horizontal", shade=35))["polarity"] == "dark"


def test_ignores_the_lateral_it_is_following():
    """A vertical tube is the row we are ON, not the one we are looking for."""
    assert not detect_crossing(with_line("vertical"))["found"]


def test_bare_soil_finds_nothing():
    assert not detect_crossing(soil(7))["found"]


def test_nearness_grows_as_the_tube_gets_closer():
    """The camera looks down at the ground ahead, so a LOWER line in frame is nearer.
    The traverse turns onto the tube when nearness crosses a threshold, so the
    ordering here is what makes that decision meaningful."""
    far  = detect_crossing(with_line("horizontal", y=80))
    near = detect_crossing(with_line("horizontal", y=210))
    assert far["found"] and near["found"]
    assert near["nearness"] > far["nearness"]
    assert 0.0 <= far["nearness"] <= 1.0 and 0.0 <= near["nearness"] <= 1.0


def test_the_two_detectors_are_complementary():
    """Each must reject the other's target, or the row change cannot work: the robot
    would either turn onto the row it is already following, or never leave it."""
    horiz, vert = with_line("horizontal"), with_line("vertical")
    assert detect_crossing(horiz)["found"] and not detect_crossing(vert)["found"]
    assert detect_tube(vert)["found"] and not detect_tube(horiz)["found"]
