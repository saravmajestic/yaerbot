"""Emitter-to-punch geometry — the seeder cannot see where it plants.

The punch tip is 16cm behind the front wheel centre and the camera's nearest visible ground
is 23cm ahead of that wheel, so the visible strip is 39-61cm AHEAD OF THE PUNCH. An emitter
under the tip is never in view. Before this was worked out, the robot stopped the moment an
emitter was "in reach" and planted 30-40cm short of every one.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "console", "python"))

# The geometry is pure arithmetic, so mirror the constants rather than importing main.py
# (which needs the Arduino bridge). If these drift from main.py the test is worthless, so
# the numbers are asserted against the source below.
NEAR, FAR = 0.39, 0.61


def ground_m(y_frac):
    return FAR - max(0.0, min(1.0, y_frac)) * (FAR - NEAR)


def test_constants_match_main():
    """Guard against the geometry being changed in one place only."""
    src = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "console", "python", "main.py")).read()
    assert "_PUNCH_TO_FRAME_NEAR_M = %.2f" % NEAR in src
    assert "_PUNCH_TO_FRAME_FAR_M  = %.2f" % FAR in src


def test_an_emitter_at_the_bottom_of_frame_is_still_ahead_of_the_punch():
    """The whole point: even the NEAREST visible emitter needs a creep, because the punch is
    behind the camera's field of view. A zero creep here would mean planting short."""
    assert ground_m(1.0) == NEAR
    assert ground_m(1.0) > 0.30, "the punch is well behind the near edge — creep is required"


def test_creep_shrinks_as_the_emitter_approaches():
    """Monotonic, so waiting longer always means a shorter creep and less error."""
    assert ground_m(0.0) > ground_m(0.5) > ground_m(1.0)


def test_creep_is_bounded_by_the_measured_frame_edges():
    for yf in (-1.0, 0.0, 0.3, 0.55, 1.0, 2.0):
        assert NEAR <= ground_m(yf) <= FAR, "a mapped distance outside the measured strip"


def test_reach_line_gives_a_sane_creep():
    """_EMIT_MIN_Y_FRAC = 0.55 is the trigger, so this is the creep the robot will usually
    perform. At the measured 0.170 m/s it must be a couple of seconds, not a lunge."""
    d = ground_m(0.55)
    assert 0.40 < d < 0.55
    assert 2.0 < d / 0.170 < 4.0
