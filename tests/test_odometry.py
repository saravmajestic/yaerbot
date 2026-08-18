"""Visual odometry — measuring the ground instead of assuming it.

`travelled` is wall-clock x a single hand-tuned _DRIP_SPEED_MPS, and it gates end-of-row. Two
observation-based calibrations of that constant disagreed by 40% (0.161 vs 0.225 m/s) purely
because the robot stuttered more in one run, and a run told to cover 5m covered about 7m.

These tests use SYNTHETIC translations, because the property being checked is that a known shift
comes back as the right distance — which real frames cannot tell you without ground truth, and
ground truth at creep duty is exactly what does not exist yet.
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "console", "python"))

from vision.odometry import FlowOdometer, PX_PER_M_DEPTH     # noqa: E402


def _texture(seed=0, h=240, w=320):
    """Soil-like texture: phase correlation needs something to correlate."""
    rng = np.random.default_rng(seed)
    base = rng.integers(60, 200, (h // 4, w // 4), dtype=np.uint8)
    return cv2.cvtColor(cv2.resize(base, (w, h), interpolation=cv2.INTER_CUBIC),
                        cv2.COLOR_GRAY2BGR)


def _shift_down(img, dy):
    """Move the ground DOWN the frame by dy px — what driving forward looks like."""
    M = np.float32([[1, 0, 0], [0, 1, dy]])
    return cv2.warpAffine(img, M, (img.shape[1], img.shape[0]), borderMode=cv2.BORDER_REFLECT)


def test_a_known_shift_comes_back_as_the_right_distance():
    """The core contract. 1090 px per metre of depth, measured with the 16mm tube as a ruler."""
    odo = FlowOdometer()
    base = _texture()
    odo.update(base)
    for _ in range(10):
        base = _shift_down(base, 10)
        odo.update(base)
    expected = 10 * 10 / PX_PER_M_DEPTH          # 10 frames x 10px
    assert odo.distance_m == __import__("pytest").approx(expected, rel=0.25), \
        "10 shifts of 10px should read ~%.3fm, got %.3fm" % (expected, odo.distance_m)


def test_a_stationary_robot_accumulates_almost_nothing():
    """THE PROPERTY THAT MATTERS MOST for integrating distance: a parked robot must not drift.
    Measured on real frames, stationary pairs read 0.06px — the dense pass had manual stops for
    course correction and they show up cleanly as zero flow."""
    odo = FlowOdometer()
    base = _texture(1)
    for _ in range(30):
        odo.update(base.copy())
    assert odo.distance_m < 0.01, \
        "a parked robot accumulated %.3fm of phantom travel over 30 frames" % odo.distance_m


def test_faster_motion_reads_as_more_distance():
    far, near = FlowOdometer(), FlowOdometer()
    a = b = _texture(2)
    far.update(a); near.update(b)
    for _ in range(8):
        a = _shift_down(a, 20); far.update(a)
        b = _shift_down(b, 5);  near.update(b)
    assert far.distance_m > 2 * near.distance_m, \
        "20px/frame (%.3fm) must read well above 5px/frame (%.3fm)" % (
            far.distance_m, near.distance_m)


def test_an_untrustworthy_correlation_is_rejected_not_guessed():
    """A bad match integrated anyway accumulates silently into the number the run is gated on —
    which is the exact failure this class exists to remove. Featureless frames give no reliable
    shift, so they must contribute nothing rather than a guess."""
    odo = FlowOdometer()
    flat = np.full((240, 320, 3), 128, np.uint8)
    odo.update(flat)
    for _ in range(10):
        odo.update(flat.copy())
    assert odo.distance_m < 0.01
    # and an implausibly large shift must be refused rather than believed
    odo2 = FlowOdometer()
    t = _texture(3)
    odo2.update(t)
    odo2.update(_shift_down(t, 200))          # far beyond MAX_SHIFT_PX
    assert odo2.distance_m == 0.0, "a 200px jump was integrated as real travel"
    assert odo2.rejected >= 1


def test_reset_clears_the_accumulator_and_the_reference_frame():
    """A second run must not inherit the first run's distance — the same class of bug that made
    a second run resume the previous run's `travelled`."""
    odo = FlowOdometer()
    t = _texture(4)
    odo.update(t)
    for _ in range(5):
        t = _shift_down(t, 10)
        odo.update(t)
    assert odo.distance_m > 0
    odo.reset()
    assert odo.distance_m == 0.0 and odo.updates == 0

    # The first update after a reset must RE-SEED, not correlate against the pre-reset frame.
    # Two earlier versions of this assertion passed with the reference frame left in place:
    # checking "distance is still 0" and checking `updates == 0` against an UNRELATED texture
    # both pass, because unrelated frames do not correlate and the stale match gets rejected
    # for low response. So continue the SAME texture — then a stale reference really would
    # correlate, and only a cleared one gives updates == 0.
    t = _shift_down(t, 10)
    odo.update(t)
    assert odo.updates == 0, \
        "the first frame after reset was correlated against the pre-reset reference"
    assert odo.distance_m == 0.0
    # and the NEXT one must work normally, or reset has broken the odometer instead
    t = _shift_down(t, 10)
    odo.update(t)
    assert odo.updates == 1 and odo.distance_m > 0, "reset left the odometer dead"


def test_it_never_raises_on_a_malformed_frame():
    """It sits in the control loop. It may degrade, it may not throw."""
    odo = FlowOdometer()
    odo.update(_texture(6))
    for bad in (np.zeros((10, 10, 3), np.uint8),          # wrong size -> re-seed
                np.zeros((240, 320), np.uint8)):          # already grey
        odo.update(bad)                                   # must not raise


def test_the_depth_scale_matches_the_console():
    """The two must not drift apart: main.py converts crossing-band movement with the same
    figure, and both come from measuring the 16mm tube as a ruler (0.76 mm/px at the frame
    bottom, 1.23 at the top, 1.19 mean implied by a 22cm strip)."""
    from test_console_imports import _load_main
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    m = _load_main()
    assert m._PX_PER_M_DEPTH == PX_PER_M_DEPTH
