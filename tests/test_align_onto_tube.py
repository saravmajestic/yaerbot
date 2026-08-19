"""SUITE 3 (part 1): turning onto the next lateral, replayed against the frames that broke it.

THE FAILURE THIS IS BUILT FROM. On 2026-08-19 the robot completed a traverse, found the next
lateral, pivoted onto it — and then drove back down the row it had just finished. The saved align
sequence from that turn is the fixture set here, and measuring it explains the whole thing:

    frame                        found      x   off centre   reject
    lat2_align0                  False      -        -       line-fit-2-of-4
    lat2_align1                  False      -        -       line-fit-2-of-4
    lat2_align2                  True      97      -63       None        <- FALSE POSITIVE
    lat2_align3                  False      -        -       line-fit-2-of-4
    lat2_align4                  False      -        -       line-fit-2-of-4
    lat2_align5                  True     162       +2       None        <- real, centred
    lat2_found                   True      80      -80       None        <- the CROSSING lateral

align2 contains no tube at all — it is bare soil and straw — and detect_tube reported one at
strength 3.4, passing every shape gate. The old loop acted on it, then escalated its nudges
(0.30 -> 0.48 -> 0.75s) with nothing bounding the sum: 137 degrees of uncommanded rotation that run,
and up to 376 degrees possible. Meanwhile the PREVIOUS lateral — a real tube, correctly shaped, just
in the wrong place — sits about 279px off centre at a 0.50m row gap and sweeps through the view on
any sideways slip.

So there are two distinct failures and they need two distinct defences:

    isolated soil artefact (align2, 63px off)  -> AGREEMENT across frames; it is not there twice
    the previous lateral   (~279px off)        -> POSITION; it is real, so only WHERE it is differs

Neither defence catches the other's case. That is why both exist, and this file holds both honest.
"""
import os
import sys

import cv2
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "console", "python"))

from test_console_imports import _load_main            # noqa: E402

FRAMES = os.path.join(_HERE, "frames", "align")

# The 02:33 turn, in the order the robot saw them.
SEQUENCE = ["lat2_align0_023313_081.jpg", "lat2_align1_023313_785.jpg",
            "lat2_align2_023314_684.jpg", "lat2_align3_023315_853.jpg",
            "lat2_align4_023317_014.jpg", "lat2_align5_023318_192.jpg"]


def _load(name):
    p = os.path.join(FRAMES, name)
    assert os.path.exists(p), "align fixture missing: %s" % p
    im = cv2.imread(p)
    assert im is not None, "could not decode %s" % p
    return im


def _grabber(names):
    """A _align_look-compatible frame source that yields these frames in order, then repeats
    the last one (a stalled camera view, which is the honest thing to do at the end)."""
    frames = [_load(n) for n in names]
    state = {"i": 0}

    def grab():
        i = min(state["i"], len(frames) - 1)
        state["i"] += 1
        return True, frames[i]
    return grab


def test_the_fixtures_are_the_frames_we_think_they_are():
    """Guard: if these files change, every conclusion below is void."""
    mod = _load_main()
    import vision.vision as V
    got = {}
    for n in SEQUENCE:
        t = V.detect_tube(_load(n))
        got[n[:11]] = (t["found"], round(t["tube_x"]) if t["found"] else None)
    assert got["lat2_align2"][0] is True, "align2 should still reproduce the false positive"
    assert abs(got["lat2_align2"][1] - 97) <= 3
    assert got["lat2_align5"][0] is True and abs(got["lat2_align5"][1] - 162) <= 3
    for blind in ("lat2_align0", "lat2_align1", "lat2_align3", "lat2_align4"):
        assert got[blind][0] is False, "%s should find nothing" % blind


# ---------------------------------------------------------------- agreement

def test_the_isolated_false_positive_is_rejected_by_agreement():
    """align1, align2, align3 as one look: only ONE of three sees a tube, so nothing is returned.

    This is the exact three-frame window the old code would have acted on, and the reason it is
    safe now — the artefact does not survive its neighbours.
    """
    mod = _load_main()
    t, _frame, n = mod._align_look(_grabber(["lat2_align1_023313_785.jpg",
                                            "lat2_align2_023314_684.jpg",
                                            "lat2_align3_023315_853.jpg"]))
    assert t is None, "acted on an isolated false positive (n_agree=%d)" % n
    assert n <= 1


def test_a_tube_seen_on_every_frame_IS_accepted():
    """The other half: agreement must not block a real, stable tube."""
    mod = _load_main()
    t, _frame, n = mod._align_look(_grabber(["lat2_align5_023318_192.jpg"] * 3))
    assert t is not None, "rejected a tube visible on all three frames"
    assert n == 3
    assert abs(t["tube_x"] - 162) <= 3


def test_blind_frames_return_nothing_and_say_how_many_agreed():
    mod = _load_main()
    t, frame, n = mod._align_look(_grabber(["lat2_align0_023313_081.jpg"] * 3))
    assert t is None and n == 0
    assert frame is not None, "must still return a frame, so the caller can save the evidence"


# ---------------------------------------------------------------- position

def test_the_centred_tube_is_ours():
    mod = _load_main()
    t, frame, _n = mod._align_look(_grabber(["lat2_align5_023318_192.jpg"] * 3))
    assert mod._align_is_ours(t, frame.shape[1]), \
        "a tube 2px off centre must be accepted as the one we pivoted on"


def test_a_tube_where_the_previous_lateral_would_be_is_NOT_ours():
    """The failure that sent the robot back down its own row. At a 0.50m row gap the previous
    lateral is ~279px off centre; nothing about its shape or confidence distinguishes it."""
    mod = _load_main()
    w = 320
    for off in (279, -279, 120, -120, 71, -71):
        fake = {"tube_x": w / 2.0 + off}
        assert not mod._align_is_ours(fake, w), \
            "accepted a tube %+dpx off centre as ours" % off
    for off in (0, 30, -30, 69, -69):
        fake = {"tube_x": w / 2.0 + off}
        assert mod._align_is_ours(fake, w), "rejected our own tube at %+dpx" % off


def test_the_crossing_lateral_seen_during_traverse_is_not_mistaken_for_ours():
    """lat2_found is the decision frame from DURING the traverse — the lateral crossing the path,
    at x=80 (-80px). It is a real tube and a correct detection, but it is not something to align
    onto, and the position gate is what says so."""
    mod = _load_main()
    import vision.vision as V
    t = V.detect_tube(_load("lat2_found_023307_576.jpg"))
    assert t["found"], "the decision frame should still detect the crossing lateral"
    assert not mod._align_is_ours(t, 320), \
        "the crossing lateral at x=%.0f was accepted as the tube to follow" % t["tube_x"]


# ---------------------------------------------------------------- the bound

def test_the_search_rotation_is_bounded():
    """The runaway, as arithmetic. Escalating pulses can rotate 272-376 deg; the cap must hold.

    Not a mock of the loop — it walks the same pulse schedule and cap the loop uses, which is the
    part that was missing entirely.
    """
    mod = _load_main()
    pulse, spent, steps = mod._NUDGE_PULSE_S, 0.0, 0
    while spent < mod._NUDGE_MAX_TOTAL_DEG and steps <= mod._NUDGE_MAX:
        step = min(pulse, max(0.05, (mod._NUDGE_MAX_TOTAL_DEG - spent) / max(1.0, mod.CAL["tdps"])))
        spent += step * mod.CAL["tdps"]
        pulse = min(mod._NUDGE_PULSE_MAX, pulse * mod._NUDGE_GROWTH)
        steps += 1
    assert spent <= mod._NUDGE_MAX_TOTAL_DEG + 1e-6, \
        "search spent %.0f deg against a %.0f deg cap" % (spent, mod._NUDGE_MAX_TOTAL_DEG)
    assert mod._NUDGE_MAX_TOTAL_DEG <= 40, \
        "a cap above ~40 deg defeats the purpose: the field lost 137 deg this way"


def test_unbounded_escalation_would_have_spun_the_robot_right_round():
    """Pin WHY the cap exists, so nobody removes it as belt-and-braces."""
    mod = _load_main()
    pulse, total = mod._NUDGE_PULSE_S, 0.0
    for _ in range(mod._NUDGE_MAX + 1):
        total += pulse
        pulse = min(mod._NUDGE_PULSE_MAX, pulse * mod._NUDGE_GROWTH)
    assert total * 60 > 180, \
        "the old schedule should exceed a half turn at the measured 60 dps (got %.0f deg)" % (
            total * 60)


@pytest.mark.skip(reason="TODO: needs a full _turn_onto_tube harness (bus, _pivot, _nudge, _drive "
                         "stubs). The pieces it would exercise — _align_look, _align_is_ours and "
                         "the rotation cap — are each covered above against real frames.")
def test_turn_onto_tube_end_to_end_replays_the_02_33_sequence():
    """The e2e version: feed the six real frames as successive looks and assert the robot stops
    with a report instead of accepting align2 or spinning past the cap."""
