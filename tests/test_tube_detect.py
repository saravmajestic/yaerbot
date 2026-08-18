"""detect_tube's gates — the three that were wrong or missing on 2026-08-18.

All three were found by measuring 2086 CONSECUTIVE frames from one 81-second pass, plus
synthetic negatives for the cases that pass could not contain. The recall numbers moved
33% -> 86% and the longest blind stretch 6.57s -> 1.10s, so these tests exist to stop any of
the three silently reverting.
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "console", "python"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import vision.vision as V                                  # noqa: E402
from vision.vision import detect_tube                      # noqa: E402
from test_crossing import with_line, soil                   # noqa: E402


# ── 1. the ANGLE gate: what makes _MIN_BANDS=3 safe ──────────────────────────

def test_a_horizontal_line_is_never_claimed_as_the_row():
    """The one thing detect_tube must never do, or the robot turns onto the row it is already
    following and the row change cannot work.

    This is not hypothetical. A 3-of-4 robust fit is a WEAK constraint: with four scattered
    band positions there are 12 chances (6 pairs x 2 remaining) for some pair's line to pass
    within max_line_dev_px of a third point, so it accepts pure scatter most of the time. On a
    synthetic horizontal line the bands came back [280.5, 219.5, 49.0, 117.0] and a 3-inlier
    fit "succeeded" at -42 degrees. The angle gate is what rejects it.
    """
    accepted = []
    for seed in range(6):
        t = detect_tube(with_line("horizontal", seed=seed))
        if t["found"]:
            accepted.append((seed, t["angle_deg"]))

    # THE STEEP CASE IS THE ONE THAT MATTERS and it must be gone entirely: claiming a ~45deg
    # line means the detector has locked onto the crossing tube itself.
    steep = [(s, a) for s, a in accepted if abs(a) > 35]
    assert not steep, "a near-horizontal line was claimed as the row: %r" % steep

    # Some frames still pass at a SHALLOW angle, and that is not the angle gate failing — it is
    # a chance 3-of-4 fit on soil noise that happens to line up near-vertically (seed 4 gives
    # -13deg). No per-frame geometric test can reject that; _track_tube's temporal jump gate is
    # what catches it, because noise does not stay put from frame to frame. So the honest
    # assertion is that this is the minority, not that it never happens.
    assert len(accepted) <= 2, \
        "%d of 6 horizontal-line frames claimed a tube: %r" % (len(accepted), accepted)


def test_the_angle_gate_names_itself_when_it_fires():
    """Driven with a tight limit on a frame that DOES fit, so it tests the gate itself rather
    than which stage happens to reject first. At _MIN_BANDS=4 the synthetic horizontal line is
    caught earlier by the band fit, so keying this on that frame tested the wrong thing."""
    t = detect_tube(with_line("vertical"), max_angle_deg=0.01)
    assert t["found"] is False
    assert t["reject"] is not None
    assert t["reject"].startswith("angle-"), \
        "expected an angle rejection, got %r" % t["reject"]


def test_a_near_vertical_tube_is_accepted():
    """The gate must not be so tight it rejects the row. Measured over 1919 real accepted
    frames the tube sits at p50 6.7deg, p95 23.6deg, p99 34.6deg from vertical, so 35 keeps
    99.1% of them."""
    t = detect_tube(with_line("vertical"))
    assert t["found"], "a vertical tube must pass (reject=%r)" % t.get("reject")
    assert abs(t["angle_deg"]) < 35


def test_the_angle_limit_is_configurable_and_actually_applied():
    frame = with_line("vertical")
    assert detect_tube(frame)["found"]
    # an absurdly tight limit must reject even a perfect vertical, or the gate is not wired up
    t = detect_tube(frame, max_angle_deg=0.01)
    assert not t["found"] and t["reject"].startswith("angle-")


# ── 2. `width` is bimodal and must not be gated on ───────────────────────────

def test_width_and_width_fwhm_are_different_measurements():
    """`width` is |partner-peak|*2 when pair-centring succeeded and the FWHM when it did not,
    AND it comes from the whole-frame profile, which smears when the tube is diagonal. So it
    collapses exactly when the robot is off-heading and most needs to steer.

    Measured over 2086 consecutive frames: width p10 14 / median 44 (bimodal), width_fwhm
    median 14 — and 14 agrees with measuring the 16mm tube as a ruler (13-21px across). That
    mis-gate discarded 321 of 1010 good detections for a whole field session.
    """
    t = detect_tube(with_line("vertical"))
    assert t["found"]
    for key in ("width", "width_fwhm", "paired"):
        assert key in t, "%s must be reported so the two can never be confused again" % key
    assert isinstance(t["width_fwhm"], int)
    assert t["width_fwhm"] > 0


def _diag(dx, seed=0, shade=35):
    """A tube leaning dx pixels across the frame height — 0 is perfectly vertical."""
    f = soil(seed)
    cv2.line(f, (160 - dx, 0), (160 + dx, 239), (shade,) * 3, 9)
    return f


def test_a_tube_is_accepted_regardless_of_what_width_reports():
    """THE REGRESSION GUARD for the width-gate bug, and the reason `width` must never be gated
    on. Measured on these exact fixtures:

        lean   angle   width  fwhm    old `width >= 30` gate
          0     0.0      11    11     REJECTS a PERFECTLY VERTICAL TUBE
         20     9.5      46    35     passes
         40    11.6      28    20     REJECTS
         60    33.4      28    21     REJECTS

    Three of four are thrown away, including the ideal case, because `width` is computed from
    the whole-frame profile and that smears as soon as the tube leans. Mutation testing showed
    the rest of this suite could not catch a revert to gating on `width`, because every other
    fixture happens to sit above 30.
    """
    for dx, ang in ((0, 0.0), (40, 11.6), (60, 33.4)):
        t = detect_tube(_diag(dx))
        assert t["found"], "lean %d (%.1fdeg) rejected: %r" % (dx, ang, t.get("reject"))
        assert t["width_fwhm"] >= 5, \
            "lean %d: fwhm %s is below the floor" % (dx, t["width_fwhm"])

    # and the thing that makes the guard meaningful: `width` really is below the old threshold
    narrow = [dx for dx in (0, 40, 60) if (detect_tube(_diag(dx))["width"] or 0) < 30]
    assert narrow, \
        "no fixture now reports width<30, so this test no longer guards the bug it was " \
        "written for — re-derive it before trusting it"


def test_the_console_gate_never_rejects_a_leaning_tube_on_ITS_WIDTH():
    """Same fixtures through main.py's real gate. The property being guarded is narrow and
    deliberate: no lean may be rejected for its WIDTH, because that was the bug.

    These fixtures DO get rejected at leans 40 and 60 — but on `sigma-2.4<2.5`, and that is a
    THIRD instance of the same root cause worth recording: `strength`, like `width`, is taken
    from the whole-frame profile, so it also decays as the tube leans (the removed axis test
    failed for exactly this reason — a diagonal tube smeared to 3.4 sigma). On synthetic soil the
    contrast is marginal enough for that to cross the threshold.

    It is LATENT rather than active: across the 2086 real frames, sigma rejected nothing (the
    rejections were line-fit 165, fwhm 101, angle 5). Fixing it properly means reporting
    strength from the BANDS rather than the whole-frame collapse, which needs its own
    measurement pass. Recorded here so it is not rediscovered from scratch.
    """
    from test_console_imports import _load_main
    m = _load_main()
    for dx in (0, 20, 40, 60):
        t = detect_tube(_diag(dx))
        if not t["found"]:
            continue                      # the angle gate may legitimately reject the steepest
        why = m._tube_reject(t)
        assert why is None or why.startswith("sigma-"), \
            "lean %d rejected on something other than contrast: %s (width=%s fwhm=%s)" % (
                dx, why, t["width"], t["width_fwhm"])
        assert why is None or not why.startswith("fwhm-"), \
            "lean %d rejected on WIDTH — the exact bug this guards" % dx


def test_fwhm_actually_measures_the_feature_and_is_not_a_constant():
    """fwhm has to MEASURE something, or gating on it is no better than gating on `width`.

    Mutation testing caught this gap: replacing the FWHM computation with a hardcoded 44 passed
    the entire suite, because every fixture clears the floor of 5. So assert it tracks the tube's
    real thickness — which is also what makes it comparable to the 16mm-tube ruler measurement
    (13-21px) that established it as the physical number.
    """
    # Measured: fwhm comes back as thickness+2 across the detectable range (9->11, 13->15,
    # 17->19, 21->23). Below 9px the line is not detected at all, so the fixtures start there.
    widths = {}
    for thick in (9, 13, 17, 21):
        f = soil(0)
        cv2.line(f, (160, 0), (160, 239), (35,) * 3, thick)
        t = detect_tube(f)
        assert t["found"], "thickness %d not detected" % thick
        widths[thick] = t["width_fwhm"]
    assert widths[9] < widths[13] < widths[17] < widths[21], \
        "fwhm does not track tube thickness: %r — it is not measuring the feature" % widths
    # and the contrast with `width`, which leaves the FWHM scale entirely once pairing kicks in
    f = soil(0)
    cv2.line(f, (160, 0), (160, 239), (35,) * 3, 21)
    t = detect_tube(f)
    assert t["width"] >= 1.8 * t["width_fwhm"], \
        "the two measurements have converged (%s vs %s) — if that is now genuinely true, this " \
        "whole gate can be simplified, but verify it before assuming" % (
            t["width"], t["width_fwhm"])


def test_fwhm_is_reported_even_on_a_rejected_frame():
    """The diagnostic has to survive rejection or the log cannot say why a frame was dropped —
    which is the whole reason the width bug lived so long."""
    t = detect_tube(with_line("horizontal", seed=0))
    assert not t["found"]
    assert t["width_fwhm"] is not None
    assert t["strength"] > 0


# ── 3. every rejection names the stage that rejected it ──────────────────────

def test_every_rejection_path_sets_a_reason():
    """Returning a bare found=False is what forced the operator's log to read "tube lost" with
    the reason left to be reconstructed from numbers printed beside it."""
    seen = set()
    frames = [with_line("horizontal", seed=s) for s in range(6)]
    frames += [soil(s) for s in range(6)]
    frames.append(np.zeros((240, 320, 3), np.uint8))          # featureless
    for f in frames:
        t = detect_tube(f)
        if not t["found"]:
            assert t.get("reject"), "a rejected frame carried no reason"
            seen.add(t["reject"].split("-")[0])
    assert seen, "no frame was rejected — the fixtures are wrong, not the code"
    assert seen <= {"no", "bands", "line", "angle"}, "unexpected reject kinds: %s" % seen


def test_an_accepted_frame_has_reject_none():
    t = detect_tube(with_line("vertical"))
    assert t["found"] and t["reject"] is None


# ── 4. _MIN_BANDS stays where the measurement put it ─────────────────────────

def test_min_bands_is_three_because_four_cannot_follow_the_tube():
    """This constant went 4 -> 3 -> 4 -> 3 in one day. The field settled it.

    At _MIN_BANDS=4, run 12:15-12:16 (36 seconds):
        travelled 0.23m           FROZEN across all 8 log windows
        rejects: 97 per 5s        line-fit-3-of-4 = 100%
        tube held 0% of frames
    The robot drove 23cm on its last good reading and never moved again. And the rejected frames
    are not marginal: lost_121524.jpg has the tube dead centre with ALL FOUR bands on it at
    [178.0, 173.5, 197.0, 159.0] — a 38px spread across a ~35px tube, which is per-band position
    noise, not a detection failure.

    The cost of 3 is real but bounded: at 10:44 it took the wrong three of four bands and steered
    off the row. That was THE FIRST FRAME of a run, tube already at the frame edge, no previous
    position to check against. In steady following the jump gate rejects a 158px leap outright.

    A robot that sometimes wanders beats one that never moves. See vision.py for the tie-break
    that would remove the trade-off entirely.
    """
    assert V._MIN_BANDS == 3, \
        "4 was tried in the field and could not follow the tube at all — 100% of frames " \
        "rejected as line-fit-3-of-4. See the comment in vision.py."


def test_a_competing_vertical_feature_is_refused_at_four_bands():
    """The field failure, reduced to a fixture: one band on the real tube, three on a brighter
    competing edge. Four bands must refuse it; three would fit the wrong three."""
    f = soil(0)
    cv2.line(f, (136, 0), (110, 239), (35,) * 3, 9)         # the real tube, left of centre
    cv2.rectangle(f, (270, 0), (319, 239), (250,) * 3, -1)  # blown-out patch on the right
    t = detect_tube(f)
    if t["found"]:
        assert t["x_near"] < 200, \
            "locked onto the bright patch at x=%.0f instead of the tube near 120" % t["x_near"]


def test_soil_with_no_tube_is_mostly_rejected():
    """Not all of it — a chance 3-of-4 fit on noise at a shallow angle can pass, and that is
    what _track_tube's temporal jump gate is for. But it must not be the common case."""
    found = sum(1 for s in range(12) if detect_tube(soil(s))["found"])
    assert found <= 4, "%d/12 bare-soil frames claimed a tube" % found
