"""How many emitters a lateral plants, simulated over the REAL frame geometry.

THE BUG THIS EXISTS FOR. On 2026-08-19 a drip dry run drove 5.01m down a lateral whose emitters sit
0.40m apart — about 12 of them — and planted exactly ONE. The re-arm condition was:

    if not in_view: armed = True        # in_view = an emitter anywhere in frame

That worked on the Logitech C310, which saw a 22cm strip of ground: at 40cm spacing there were long
stretches with no emitter in view, so `armed` came back between emitters. The QHM-999RL sees
**0.43m**, which is WIDER than the 0.40m spacing, so an emitter is essentially always somewhere in
frame, `in_view` never went False, and `armed` never returned after the first plant.

Nothing about the logic changed. The lens did. That is the whole lesson: a constant expressed in
pixels or in "frames" silently changes meaning when the camera changes, and only a test written in
GROUND UNITS catches it.

Fixed by re-arming when the emitter leaves the REACH ZONE rather than the frame, with
_min_replant() (0.20m at a 0.40m spacing) as the guard against double-planting one emitter.

NOTE ON THE SIMULATIONS BELOW: they must set `armed = False` after a plant, because the camera loop
does that itself once the plant completes. The first version of this file left it out, so `armed`
stayed True throughout and every simulation "passed" while proving nothing — including the one
meant to REPRODUCE the bug, which reported 12 plants where the field saw 1. A simulation of a state
machine has to include every transition or it is testing a different machine.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from test_console_imports import _load_main            # noqa: E402

# Measured on the robot, 2026-08-19
CAM_FOV_M = 0.43           # ground covered top to bottom of frame
REACH_FRAC = 0.45          # bottom 45% of frame acts (1 - _EMIT_MIN_Y_FRAC of 0.55)
REACH_M = CAM_FOV_M * REACH_FRAC


def _drive_lateral(mod, length_m, gap_m, step_m=0.01, fov_m=CAM_FOV_M):
    """Walk a lateral in `step_m` increments and count plants, using the real _emit_debounce.

    An emitter at ground position p is IN VIEW while the robot is within fov_m behind it, and IN
    REACH while it is within the bottom REACH_FRAC of that view. Both are expressed in metres of
    ground, which is what makes this test survive a camera change.
    """
    mod._drip["emitter_gap"] = gap_m
    emitters = [gap_m * (i + 1) for i in range(int(length_m / gap_m))]

    armed, last_plant_at, plants = True, -1e9, []
    travelled = 0.0
    while travelled < length_m:
        # nearest emitter still ahead of or level with the camera's near edge
        in_view = any(travelled <= p <= travelled + fov_m for p in emitters)
        in_reach = any(travelled <= p <= travelled + REACH_M for p in emitters)
        plant, armed = mod._emit_debounce(in_reach, armed, travelled, last_plant_at)
        if plant:
            plants.append(round(travelled, 3))
            last_plant_at = travelled
            armed = False          # the loop does this after the plant completes (main.py)
        travelled += step_m
    return plants, emitters, in_view


def test_a_five_metre_lateral_plants_every_emitter_not_just_one():
    """THE FIELD FAILURE, as a test. 5m at 0.40m spacing = 12 emitters; the run planted 1."""
    mod = _load_main()
    plants, emitters, _ = _drive_lateral(mod, 5.0, 0.40)
    assert len(plants) >= len(emitters) - 1, \
        "planted %d of %d emitters over 5m: %r" % (len(plants), len(emitters), plants)
    # and not wildly over-planting either
    assert len(plants) <= len(emitters) + 1, \
        "planted %d for %d emitters — double-counting: %r" % (
            len(plants), len(emitters), plants)


def test_no_emitter_is_planted_twice():
    """_min_replant() is the guard that lets the reach-zone re-arm be safe. Consecutive plants must
    never be closer than it, or one emitter is being counted twice — the 03:33 failure (stops at
    2.73m and 2.84m, 11cm apart on a 40cm line)."""
    mod = _load_main()
    plants, _, _ = _drive_lateral(mod, 5.0, 0.40)
    floor = mod._min_replant()
    gaps = [round(b - a, 3) for a, b in zip(plants, plants[1:])]
    assert all(g >= floor - 1e-9 for g in gaps), \
        "two plants closer than the %.2fm floor: %r" % (floor, gaps)


def test_the_wide_view_is_exactly_what_broke_it():
    """Pin the cause, so nobody 'fixes' the re-arm back to frame-exit.

    Re-arming on FRAME exit gives one plant when the view is wider than the spacing, and the right
    number when it is narrower. That asymmetry is the bug, reproduced here directly.
    """
    mod = _load_main()
    gap = 0.40

    def frame_exit_count(fov_m):
        mod._drip["emitter_gap"] = gap
        emitters = [gap * (i + 1) for i in range(int(5.0 / gap))]
        armed, last, n, travelled = True, -1e9, 0, 0.0
        while travelled < 5.0:
            in_view = any(travelled <= p <= travelled + fov_m for p in emitters)
            in_reach = any(travelled <= p <= travelled + fov_m * REACH_FRAC for p in emitters)
            if not in_view:                     # THE OLD, BROKEN CONDITION
                armed = True
            if in_reach and armed and travelled - last >= mod._min_replant():
                n += 1
                last = travelled
                armed = False      # as the loop does
            travelled += 0.01
        return n

    assert frame_exit_count(0.22) > 5, \
        "on the C310's 22cm view the old condition worked, so the test set-up is wrong"
    assert frame_exit_count(0.43) == 1, \
        "the QHM's 43cm view must reproduce the one-emitter failure under the OLD condition"


def test_a_wider_spacing_than_the_view_still_works():
    """The fix must not depend on the view being wider than the spacing — a 0.6m spacing on a 0.43m
    view is the other regime, and both must plant every emitter."""
    mod = _load_main()
    plants, emitters, _ = _drive_lateral(mod, 4.8, 0.60)
    assert len(plants) >= len(emitters) - 1, "planted %d of %d at 0.60m spacing" % (
        len(plants), len(emitters))


def test_a_missed_emitter_does_not_break_the_next_one():
    """_min_replant() is a floor, never a trigger: skipping one emitter must not shift or block the
    rest, or a single model miss would cascade down the whole lateral."""
    mod = _load_main()
    mod._drip["emitter_gap"] = 0.40
    emitters = [0.4, 0.8, 1.6, 2.0]        # 1.2 deliberately missing
    armed, last, plants, travelled = True, -1e9, [], 0.0
    while travelled < 2.4:
        in_reach = any(travelled <= p <= travelled + REACH_M for p in emitters)
        plant, armed = mod._emit_debounce(in_reach, armed, travelled, last)
        if plant:
            plants.append(round(travelled, 2))
            last = travelled
            armed = False
        travelled += 0.01
    assert len(plants) == len(emitters), "planted %d of %d with a gap in the line: %r" % (
        len(plants), len(emitters), plants)
