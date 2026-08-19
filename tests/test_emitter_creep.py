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
# FAR CORRECTED 2026-08-19: was 0.61, which was a C310 number — its comment in main.py recorded
# "bottom of frame -> top of frame 22 cm", the old camera's strip. The QHM-999RL sees 43 cm, so
# punch -> far edge is 0.39 + 0.43 = 0.82. The error only mattered once the retrained model started
# detecting away from the very bottom of frame, where the two values agree: it under-estimated the
# distance by 7 cm at y=162, 10 cm at mid-frame and 18 cm at y=30.
NEAR, FAR = 0.39, 0.82


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


def test_reach_line_distance_is_within_the_visible_strip():
    """_EMIT_MIN_Y_FRAC = 0.55 is where an observation becomes trustworthy enough to queue, so this
    is the distance the robot typically commits from.

    REWRITTEN 2026-08-19. It used to assert 0.40 < d < 0.55 and "2-4 seconds of creep", both of which
    are dead: the creep is gone (the conveyor drives the distance WHILE WATCHING instead of blind —
    see _emit_queue in main.py), and with FAR corrected from the C310's 0.61 to 0.82 the distance at
    this line is 0.583 m rather than 0.50 m. What remains true, and worth pinning, is that it lies
    inside the visible strip and is comfortably past the punch.
    """
    d = ground_m(0.55)
    assert NEAR < d < FAR, "%.3f is outside the visible strip %.2f-%.2f" % (d, NEAR, FAR)
    assert d > NEAR + 0.10, ("committing only %.0f mm past the punch leaves no room for the "
                             "conveyor to refine the estimate" % ((d - NEAR) * 1000))
