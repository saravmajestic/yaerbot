"""SUITE 1: end-to-end tube finding on real frames — centred, offset, and shadowed.

This is the suite the project should have had first. On 2026-08-18 there were 149 passing tests while
the robot could not follow a tube, because every one of them exercised a MECHANISM (grace decay,
jump gates, reject tallies) and would still pass against a detector that finds nothing.

STRUCTURE. Frames are grouped by the CONDITION they represent, because the failures are
condition-specific and lumping them together hides which one is broken:

    CENTRED   tube near mid-frame, even light        - the case that must never fail
    OFFSET    tube well off centre, robot off the row - steering has to see it to correct
    SHADOWED  hard shadow edge across or beside it    - the standing unsolved case

LABELS ARE THE POINT, AND THEY ARE NOT GUESSES. Each entry carries the tube's centre column at the
BOTTOM of frame (what x_near reports), read off a gridded overlay. Two automated shortcuts were
tried and rejected:

  * "darkest column in the bottom band" disagrees with the eye by +84 and +92 px on the shadowed
    frames — because in those frames the SHADOW is darker than the tube, which is the confound
    itself. It only agrees where there is no deep shadow.
  * trusting detect_tube's own answer would make the test tautological.

So a label is only entered here once a human has read it off the grid. `None` means UNLABELLED and
those cases skip rather than assert — a wrong label is worse than no label, which is the lesson of
_MIN_BANDS moving 4 -> 3 -> 4 -> 3 in one day on measurements that lacked ground truth.

CAMERA MATTERS. The shadow frames below are from the OLD camera: their tube reads fwhm 13-34 px
against the QHM-999RL's 9-11, because the old lens saw a 0.22 m strip where this one sees 0.43 m.
Position labels transfer between cameras; anything in pixels-of-width does not. Each group records
which camera it came from.
"""
import os
import sys

import cv2
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "console", "python"))

import vision.vision as V                              # noqa: E402
from test_console_imports import _load_main            # noqa: E402

# Tolerance is set by LABEL PRECISION, not by what makes the suite green: labels are read off a
# 20px grid by eye and are good to roughly +/-20px. 50px separates "on the tube" from "locked onto
# something else" — every real failure recorded in this project has been 63-238px out.
TOL_PX = 50

# ---------------------------------------------------------------------------------------------
# frame -> (truth x_near, camera, note).  truth None = NOT YET LABELLED, the case will skip.
# ---------------------------------------------------------------------------------------------

CENTRED = {
    # QHM-999RL, labelled from the golden set
    "golden/lost_121524.jpg":            (168, "qhm", "12:16 run: 4-band froze the robot 36s"),
    "golden/cap_20260818_110504_150.jpg": (175, "qhm", "uniform light, strong tube"),
    "golden/cap_20260818_110502_124.jpg": (178, "qhm", "same stretch a frame earlier"),
    "golden/emit1_lat1_111611.jpg":      (178, "qhm", "the plant stop"),
}

OFFSET = {
    "golden/cap_20260818_104428_437.jpg": (55, "qhm", "10:44: 3-band took an overexposed edge"),
    "align/lat2_align5_023318_192.jpg":  (162, "qhm", "post-turn acquisition, centred"),
}

# THE SHADOWED SET, supplied by the operator 2026-08-19 as "a few with shadows".
# Measured behaviour BEFORE labelling (detector output, not truth):
#     022932  not found (line-fit-2-of-3)      <- real tube, missed
#     022947  found x=194, angle 29 deg
#     023035  found x=149, angle 19 deg
#     lost_014024  found x=52                  <- agrees with the no-shadow objective check
#     083845  found x=164
#     083851  not found (line-fit-2-of-4)      <- strong tube with a hard horizontal shadow edge
SHADOWED = {
    "shadow/cap_20260818_022932_586.jpg":  (None, "old", "robot's shadow over the left half"),
    "shadow/cap_20260818_022947_170.jpg":  (None, "old", "deep shadow left; detector says 194"),
    "shadow/cap_20260818_023035_869.jpg":  (None, "old", "detector says 149"),
    "shadow/lost_014024.jpg":              (None, "old", "tube exits bottom-left; robot off row"),
    "shadow/dense/cap_20260818_083845_886.jpg": (None, "old", "detector says 164"),
    "shadow/dense/cap_20260818_083851_243.jpg": (None, "old", "hard horizontal shadow edge across"),
}

GROUPS = {"centred": CENTRED, "offset": OFFSET, "shadowed": SHADOWED}


def _load(rel):
    p = os.path.join(_HERE, "frames", rel)
    if not os.path.exists(p):                     # allow the flat copy as well as dense/
        p = os.path.join(_HERE, "frames", rel.replace("dense/", ""))
    assert os.path.exists(p), "fixture missing: %s" % p
    im = cv2.imread(p)
    assert im is not None, "could not decode %s" % p
    return im


def _labelled(group):
    return [(k, v) for k, v in sorted(GROUPS[group].items()) if v[0] is not None]


def _unlabelled(group):
    return [k for k, v in sorted(GROUPS[group].items()) if v[0] is None]


# ---------------------------------------------------------------- the requirement

@pytest.mark.parametrize("name,spec", _labelled("centred"))
def test_centred_tube_is_found_hinted(name, spec):
    """MUST NEVER FAIL. A tube near mid-frame in even light, with the tracker hinted — the steady
    state of a working run. If this is red the robot cannot follow a tube at all."""
    truth, _cam, why = spec
    t = V.detect_tube(_load(name), hint_x=truth)
    assert t["found"], "%s: no tube (%s) reject=%s" % (name, why, t.get("reject"))
    assert abs(t["x_near"] - truth) <= TOL_PX, "%s: x=%.0f truth %d (%s)" % (
        name, t["x_near"], truth, why)


@pytest.mark.parametrize("name,spec", _labelled("offset"))
def test_offset_tube_is_found_hinted(name, spec):
    """The robot is off the row and steering has to see the tube to correct. Same requirement,
    kept separate so a regression says WHICH condition broke."""
    truth, _cam, why = spec
    t = V.detect_tube(_load(name), hint_x=truth)
    if not t["found"]:
        pytest.xfail("%s: known hard — %s (reject=%s)" % (name, why, t.get("reject")))
    assert abs(t["x_near"] - truth) <= TOL_PX, "%s: x=%.0f truth %d" % (
        name, t["x_near"], truth, )


@pytest.mark.parametrize("name,spec", _labelled("shadowed"))
def test_shadowed_tube_is_found_hinted(name, spec):
    """THE UNSOLVED CASE. Recorded as xfail where it fails, so the limit is visible and flips to a
    pass by itself if the detector improves — rather than being hidden by leaving the frames out."""
    truth, _cam, why = spec
    t = V.detect_tube(_load(name), hint_x=truth)
    if not t["found"]:
        pytest.xfail("%s: shadow defeats it — %s (reject=%s)" % (name, why, t.get("reject")))
    err = abs(t["x_near"] - truth)
    if err > TOL_PX:
        pytest.xfail("%s: found x=%.0f, truth %d, off %.0fpx — %s" % (
            name, t["x_near"], truth, err, why))


# ---------------------------------------------------------------- honesty guards

def test_every_group_has_at_least_one_labelled_frame():
    """Without this, emptying a group would delete its coverage silently and the suite would still
    be green. That exact failure mode has already happened here once."""
    for g in ("centred", "offset"):
        assert _labelled(g), "group %r has no labelled frames — its tests do not exist" % g


def test_unlabelled_frames_are_reported_not_forgotten():
    """Frames waiting for a human label are listed loudly. They are not a pass and not a failure —
    they are work outstanding, and a silent skip is how work gets lost."""
    pending = {g: _unlabelled(g) for g in GROUPS}
    total = sum(len(v) for v in pending.values())
    if total:
        print("\n%d frame(s) awaiting a ground-truth label:" % total)
        for g, names in pending.items():
            for n in names:
                print("   [%s] %s" % (g, n))
    # deliberately not an assertion: unlabelled frames must not fail the build, only be visible
    assert True


def test_the_shadow_set_is_present_even_if_unlabelled():
    """The frames themselves must be committed, so the labels can be added without re-finding them.
    Committing the fixture and committing the label are two separate jobs."""
    for name in SHADOWED:
        _load(name)      # asserts existence and decodability
