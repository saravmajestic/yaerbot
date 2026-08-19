"""Does the robot stop ONCE per emitter, in the right place, along a whole row?

That is the operator's requirement, stated 2026-08-19: the stop COUNT must match the emitters in the
row, and each stop must be within +/-5 cm of its emitter. This file simulates a row in GROUND UNITS
and checks exactly that, because two previous attempts at this logic failed for reasons no
pixel-level or gate-level test could have caught.

THE HISTORY, which is why the simulation is written the way it is.

  ATTEMPT 1 — re-arm when no emitter is visible. The camera sees 0.43 m of ground and emitters are
  0.40 m apart, so there is essentially always one in frame: it never re-armed, and a 5 m lateral
  planted ONCE. Fixed by re-arming on leaving the reach zone.

  ATTEMPT 2 — re-arm on leaving the reach zone, then blind-creep the emitter under the punch. The
  punch sits 0.39 m behind the nearest visible ground, so the creeps came out at 0.43-0.51 m, longer
  than the 0.40 m spacing. Every stop drove past the next emitter with the camera unconsulted:
  7 stops over a row holding 13 emitter positions (the 07:18 run). Capping the creep would fix the
  count and break the placement — short by up to 14 cm against a +/-5 cm requirement.

  ATTEMPT 3, tested here — a CONVEYOR. Each observation becomes a target odometer reading
  (travelled + distance-to-punch); targets queue; the punch fires as the robot reaches each. Several
  emitters are in flight at once, which a 0.40 m spacing against a 0.39 m punch offset forces. The
  targets deduplicate themselves: as the robot advances, `travelled` grows by exactly what the gap
  shrinks, so one physical emitter yields the SAME target from every frame it appears in.

THE GEOMETRY, all measured on the robot:

    punch tip -> nearest visible ground   0.39 m      (_PUNCH_TO_FRAME_NEAR_M)
    punch tip -> farthest visible ground  0.82 m      (_PUNCH_TO_FRAME_FAR_M, corrected from 0.61)
    an observation is trusted from        y >= 0.55 h  (_EMIT_MIN_Y_FRAC) => gap <= 0.58 m
    emitter spacing in the demo row       0.40 m
"""
import os
import random
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from test_console_imports import _load_main            # noqa: E402

STEP_M = 0.01          # simulation granularity; the real loop samples far finer than this
PLACE_TOL_M = 0.05     # the operator's requirement


H_PX = 240             # frame height the geometry constants are expressed against

# THE TEST'S OWN COPY OF THE MEASURED GEOMETRY, and it must stay independent of main.py's.
# Converting distance -> row with the module's constants and then letting the module convert back
# is CIRCULAR: the error cancels, and mutation testing proved it — setting FAR back to the wrong
# C310 value of 0.61 left every test passing. These are the physical measurements:
#     punch tip -> front wheel 0.16 m, front wheel -> bottom of frame 0.23 m  => near 0.39 m
#     frame top to bottom 0.43 m (operator measured on the QHM-999RL)        => far  0.82 m
TRUE_NEAR_M = 0.39
TRUE_FAR_M = 0.82

# DETECTION ROW JITTER, from the field rather than invented. The 07:18 run re-confirmed stale boxes
# on a fresh frame and logged the row moving between two observations of the SAME emitter:
#     emitter 3: y 150 -> 108   (42 px)
#     emitter 6: y 174 -> 156   (18 px)
#     emitter 5: y 162 -> 150   (12 px)
# So +/-15 px is conservative, not pessimistic. The first version of this file used +/-2 px, which is
# a stability the model does not have — and with it, disabling the deduplication entirely still
# passed every test.
JITTER_PX = 15.0


def _drive_row(mod, emitters, row_len, miss=(), step_m=STEP_M, jitter_px=JITTER_PX):
    """Drive a row and return where the punch fired.

    ROUTED THROUGH THE REAL GEOMETRY, deliberately. An earlier version of this helper computed the
    target as `travelled + (E - travelled)` and passed the gap straight to _emit_queue. That made the
    simulation useless in two ways which mutation testing exposed:

      * it never touched _emitter_ground_m, so _PUNCH_TO_FRAME_FAR_M could be set back to the wrong
        C310 value of 0.61 and every test still passed;
      * the arithmetic was EXACT, so repeated sightings of one emitter produced bit-identical targets
        and merged whatever the dedupe tolerance was — disabling dedupe entirely still passed.

    Both are the same mistake as the odometry test that fed the odometer a perfect duplicate frame:
    a simulation with no noise cannot exercise the machinery that exists to handle noise.

    So this converts the true gap into a frame ROW, jitters it, and lets _emitter_ground_m convert it
    back — which is what the camera loop does. `miss` names emitters the model never sees, so a
    detection failure can be told apart from a logic failure.
    """
    # The TEST places emitters using the measured truth; the CODE is what converts a row back to a
    # distance. That asymmetry is the point — if the code's constants are wrong, the target is wrong.
    near, far = TRUE_NEAR_M, TRUE_FAR_M
    rng = random.Random(20260819)          # fixed seed: noisy but reproducible

    targets, plants, last_plant_at = [], [], -1e9
    travelled = 0.0
    while travelled < row_len:
        # nearest emitter still ahead of the punch and inside the visible strip
        ahead = sorted(e - travelled for e in emitters
                       if e not in miss and near <= (e - travelled) <= far)
        if ahead:
            true_gap = ahead[0]
            # ground distance -> the frame row the model would report it in
            frac = (far - true_gap) / (far - near)
            y = frac * H_PX + rng.uniform(-jitter_px, jitter_px)
            y = max(0.0, min(float(H_PX), y))
            if y >= mod._EMIT_MIN_Y_FRAC * H_PX:          # only trusted rows are queued
                # main.py's _emitter_ground_m does the row -> distance conversion, using ITS
                # constants. A wrong FAR shows up here as a wrong target and a failed placement.
                gap = mod._emitter_ground_m({"position": (160.0, y)}, H_PX)
                if gap and gap > 0.0:
                    targets = mod._emit_queue(targets, travelled, gap)
        due, rest = mod._emit_due(targets, travelled, last_plant_at)
        if due is not None:
            targets = rest
            plants.append(round(travelled, 3))
            last_plant_at = travelled
        travelled += step_m
    return plants, targets


def _pair(plants, emitters):
    """Nearest-emitter for each plant, so placement error can be reported per stop."""
    return [(p, min(emitters, key=lambda e: abs(e - p))) for p in plants]


# ---------------------------------------------------------------- the requirement

def test_one_stop_per_emitter_over_a_five_metre_row():
    """THE FIELD FAILURE, as a test. 0.40 m spacing over 5 m: attempt 1 planted once, attempt 2
    planted 7 of 13. This must plant every one."""
    mod = _load_main()
    mod._drip["emitter_gap"] = 0.40
    emitters = [round(0.40 * (i + 1), 2) for i in range(12)]     # 0.40 .. 4.80
    plants, leftover = _drive_row(mod, emitters, row_len=5.4)

    assert len(plants) == len(emitters), (
        "planted %d for %d emitters (leftover queue %s): %s"
        % (len(plants), len(emitters), [round(t, 2) for t in leftover], plants))


def test_every_stop_is_within_5cm_of_its_emitter():
    """The operator's tolerance. Placement is what the conveyor buys over a capped creep."""
    mod = _load_main()
    mod._drip["emitter_gap"] = 0.40
    emitters = [round(0.40 * (i + 1), 2) for i in range(12)]
    plants, _ = _drive_row(mod, emitters, row_len=5.4)

    bad = [(p, e, round((p - e) * 1000)) for p, e in _pair(plants, emitters)
           if abs(p - e) > PLACE_TOL_M]
    assert not bad, "stops further than %.0f cm from their emitter (plant, emitter, mm): %s" % (
        PLACE_TOL_M * 100, bad)


def test_no_emitter_is_hit_twice():
    """One target per emitter — the property the self-deduplication exists to provide."""
    mod = _load_main()
    mod._drip["emitter_gap"] = 0.40
    emitters = [round(0.40 * (i + 1), 2) for i in range(12)]
    plants, _ = _drive_row(mod, emitters, row_len=5.4)

    hit = [e for _p, e in _pair(plants, emitters)]
    dupes = sorted({e for e in hit if hit.count(e) > 1})
    assert not dupes, "planted twice on: %s (plants %s)" % (dupes, plants)


def test_a_missed_emitter_does_not_cascade():
    """A single model miss must cost exactly one emitter, not shift or block the rest. The old
    distance floor was only ever a floor, never a trigger, and that must stay true."""
    mod = _load_main()
    mod._drip["emitter_gap"] = 0.40
    emitters = [round(0.40 * (i + 1), 2) for i in range(12)]
    missed = {emitters[3], emitters[7]}
    plants, _ = _drive_row(mod, emitters, row_len=5.4, miss=missed)

    assert len(plants) == len(emitters) - len(missed), (
        "expected %d stops with %d missed, got %d: %s"
        % (len(emitters) - len(missed), len(missed), len(plants), plants))
    bad = [(p, e) for p, e in _pair(plants, emitters) if abs(p - e) > PLACE_TOL_M]
    assert not bad, "a miss displaced other stops: %s" % bad


@pytest.mark.parametrize("gap_m", [0.30, 0.40, 0.50, 0.65, 0.80])
def test_it_works_across_plausible_emitter_spacings(gap_m):
    """0.40 m is the demo row, but the spacing is an operator setting. It must not be the case that
    the logic happens to work at one value — attempt 1 broke precisely because the spacing crossed
    the camera's field of view."""
    mod = _load_main()
    mod._drip["emitter_gap"] = gap_m
    # START THE ROW BEYOND THE PUNCH'S BLIND ZONE. An emitter closer than
    # _PUNCH_TO_FRAME_NEAR_M is already BEHIND the punch before the run begins and can never be
    # planted — physically, not as a logic failure. The first version of this test placed the first
    # emitter at `gap_m`, so at 0.30 m spacing it sat inside the 0.39 m blind zone and the test
    # blamed the code. In the field the operator places the robot with the first emitter ahead of it.
    first = round(mod._PUNCH_TO_FRAME_NEAR_M + 0.05, 3)
    n = int(4.0 / gap_m)
    emitters = [round(first + gap_m * i, 3) for i in range(n)]
    plants, _ = _drive_row(mod, emitters, row_len=emitters[-1] + 0.4)

    assert len(plants) == len(emitters), "spacing %.2f m: planted %d of %d — %s" % (
        gap_m, len(plants), len(emitters), plants)
    bad = [(p, e) for p, e in _pair(plants, emitters) if abs(p - e) > PLACE_TOL_M]
    assert not bad, "spacing %.2f m: %s" % (gap_m, bad)


# ---------------------------------------------------------------- the mechanism

def test_repeated_sightings_of_one_emitter_collapse_to_one_target():
    """The self-deduplication, directly. The same emitter seen from three positions must produce one
    target, near its true position — this is the whole reason the count comes out right."""
    mod = _load_main()
    targets = []
    for travelled in (0.00, 0.05, 0.10):
        gap = 0.50 - travelled            # emitter fixed at ground 0.50
        targets = mod._emit_queue(targets, travelled, gap)
    assert len(targets) == 1, "one emitter produced %d targets: %s" % (len(targets), targets)
    assert abs(targets[0] - 0.50) < 0.01


def test_two_real_emitters_stay_separate():
    """The other side of dedupe: 0.40 m apart is far beyond the tolerance, so they must not merge."""
    mod = _load_main()
    targets = mod._emit_queue([], 0.0, 0.45)
    targets = mod._emit_queue(targets, 0.0, 0.85)
    assert len(targets) == 2, "two emitters 0.40 m apart merged: %s" % targets


def test_nothing_fires_before_the_robot_reaches_the_target():
    """A punch ahead of its target plants short — the failure the conveyor replaced."""
    mod = _load_main()
    due, rest = mod._emit_due([0.50], travelled=0.49, last_plant_at=-1e9)
    assert due is None and rest == [0.50]
    due, rest = mod._emit_due([0.50], travelled=0.50, last_plant_at=-1e9)
    assert due == 0.50 and rest == []


def test_the_queue_survives_several_emitters_in_flight():
    """A 0.40 m spacing against a 0.39 m punch offset means more than one emitter is always pending.
    If the queue could only hold one, the count would halve — which is what attempt 2 did."""
    mod = _load_main()
    mod._drip["emitter_gap"] = 0.40
    emitters = [0.40, 0.80, 1.20]
    targets = []
    for e in emitters:
        targets = mod._emit_queue(targets, 0.0, e)      # all three visible from the start
    assert len(targets) == 3, "queue collapsed %d emitters into %s" % (len(emitters), targets)
    assert targets == sorted(targets), "targets must stay ordered so the head is the next to fire"
