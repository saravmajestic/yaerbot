"""SUITE 2: does the emitter model see an emitter WHEREVER it sits in the frame?

THIS SUITE IS THE ACCEPTANCE TEST FOR THE RETRAIN. It is expected to be partly red right now, and
the mid-frame cases going green is what "the retrain worked" means.

WHAT THE FIELD MEASURED, 2026-08-19. Across five drip dry runs, every emitter observation the model
produced, as (confidence, y in a 240px frame):

    conf >= 0.90 : y = 204, 222, 210, 222     mean y 214
    conf <  0.90 : y = 150, 198, 168, 78, 210 mean y 161

EVERY confident detection sits in the bottom 15% of frame — the point where the emitter is nearest
the camera and therefore LARGEST in pixels. One run returned nothing at all on 372 consecutive
frames of tube, then fired twice at 0.96 and 0.97 in the last metre.

The arithmetic explains it. The model was trained on Logitech C310 frames covering 0.22 m of ground
over 240 px. The QHM-999RL covers 0.43 m over the same 240 px, so the same physical emitter spans
about HALF the pixels it did in training, and FOMO's 160x160 input halves it again. An emitter that
was a comfortable ~20 px blob is now ~10 px, and only reaches trainable size in the last few
centimetres of its pass through the frame.

WHY THAT MATTERS OPERATIONALLY, and why this suite is organised by y: detecting only at y=214 leaves
almost no reaction distance. The robot has to see an emitter while it is still mid-frame to stop on
it, so mid-frame recall IS the requirement, not a nicety.

RUNNING IT. The ML runner is a service on the board (127.0.0.1:1337), so these tests SKIP off-board
rather than fail. That is deliberate: a red suite that only means "wrong machine" trains people to
ignore red.

A CAVEAT ON THE FIXTURES, found the hard way. The emitN_latM frames saved during a run are NOT
necessarily the frames the model scored: _save_named saves the CURRENT frame while the detection came
from the worker's latest inference, up to 374 ms older. Proof: emit2_lat1_025342 stopped the robot at
conf 0.92 in the field, and re-running the model on that saved frame returns nothing at all. So
labels here are what a HUMAN can see in the frame, never what the log said at the time.
"""
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
# BOTH LAYOUTS. This is the one suite that can only run WITH THE BOARD, because the ML runner is a
# service on it — and the first version only added the repo path (<repo>/console/python), so on the
# board every test skipped with "No module named 'vision'". A suite that cannot run in the only place
# it is meaningful is not a suite. On the board the app is mounted at /app.
for _candidate in (os.path.join(os.path.dirname(_HERE), "console", "python"), "/app/python"):
    if os.path.isdir(os.path.join(_candidate, "vision")):
        sys.path.insert(0, _candidate)

FRAMES = os.path.join(_HERE, "frames", "emitter")
# On the board the fixtures travel with the tests, so _HERE is right there too — but fall back to the
# app's own captures directory if they were not copied, since that is where the real frames live.
if not os.path.isdir(FRAMES) and os.path.isdir("/app/captures"):
    FRAMES = "/app/captures"


def _ml():
    """The emitter model, or a skip. Imported lazily so collection works off-board."""
    try:
        from vision.emitter_ml import detect_emitter_ml, ml_available
    except Exception as e:                                   # noqa: BLE001
        pytest.skip("emitter_ml not importable here: %s" % e)
    if not ml_available():
        pytest.skip("no ML runner reachable (expected off-board; it lives on 127.0.0.1:1337)")
    return detect_emitter_ml


def _frames_in(band):
    """Fixtures for one y band, or an empty list if none are labelled yet."""
    return sorted(LABELS.get(band, {}).items())


# ---------------------------------------------------------------------------------------------
# frame -> note.  Grouped by where the emitter sits in frame, because that is the axis the model
# currently fails along. Fill these in from captures/qhm999rl-2026-08-19/ as frames are labelled;
# a band with no entries reports itself as missing coverage rather than passing silently.
#
# BANDS, in a 240px frame:
#     "near"  y >= 190   the only place the model currently works
#     "mid"   130 <= y < 190   the reach zone the robot must act in  <- THE RETRAIN TARGET
#     "far"   y < 130    useful lookahead, nice to have
# ---------------------------------------------------------------------------------------------
LABELS = {
    "near": {},
    "mid":  {},
    "far":  {},
}

MIN_CONF = 0.60          # the shipped _EMIT_CONF after the 2026-08-19 field evidence

# ---------------------------------------------------------------------------------------------
# RETRAIN BASELINE, measured 2026-08-19 against ei-deployment-version 8 on the board.
#
# This is a REGRESSION pin, not a tautology: there is independent before/after evidence that v8 is
# better than v6 on these identical frames, so "the model that works must keep working" is a real
# requirement. Re-baseline deliberately when the model is changed on purpose — never to make a red
# test green.
#
#     frame                        v6 (before)   v8 (after)   delta
#     emit2_lat1_025342_761.jpg    none          0.79 @y186   GAINED, and MID-frame
#     emit2_lat1_025653_793.jpg    none          0.66 @y210   GAINED
#     emit2_lat2_023344_413.jpg    none          0.96 @y180   GAINED, and MID-frame
#     emit3_lat1_025834_914.jpg    0.65 @y222    1.00 @y204   +0.35
#     emit3_lat1_024725_616.jpg    0.73 @y198    0.97 @y210   +0.24
#     emit2_lat1_025825_168.jpg    0.75 @y222    0.97 @y198   +0.22
#     emit3_lat1_024600_270.jpg    0.80 @y222    1.00 @y204   +0.20
#     emit2_lat1_030306_133.jpg    0.86 @y216    0.98 @y210   +0.12
#
#     gained 3, lost 0, every remaining frame more confident.
#
# THE NUMBER THAT MATTERED. Before the retrain EVERY confident detection sat at y >= 190 (mean 214) —
# the point where the emitter is nearest and largest — which left the robot almost no reaction
# distance. Over the 19 saved emitter frames v8 now detects 18, and their positions spread:
#     far <130: 1    mid 130-190: 7    near >=190: 10    none: 1
# Seven mid-frame detections against none before is the blind spot closing, and it is the reason the
# retrain was worth doing rather than another threshold tweak.
#
# AND IT DID NOT BECOME TRIGGER-HAPPY: 0 boxes on 17 frames with no emitter in them (9 bare-soil
# crossing frames, 8 align frames of which four contain no tube at all).
#
# _EMIT_CONF STAYS AT 0.60. v8's confidences on real emitters run 0.66-1.00 with a median near 0.97,
# so a higher gate looks tempting — but the two lowest (0.66, 0.79) are exactly the frames v6 missed
# entirely, so 0.60 is what is buying the recall. With zero measured false positives there is nothing
# to trade it against.
BASELINE_V8 = {
    "emit2_lat1_025342_761.jpg": 0.79,
    "emit2_lat1_025653_793.jpg": 0.66,
    "emit2_lat2_023344_413.jpg": 0.96,
    "emit3_lat1_025834_914.jpg": 1.00,
    "emit3_lat1_024725_616.jpg": 0.97,
    "emit2_lat1_025825_168.jpg": 0.97,
    "emit3_lat1_024600_270.jpg": 1.00,
    "emit2_lat1_030306_133.jpg": 0.98,
}
BASELINE_TOL = 0.15     # run-to-run variation on the same frame; a real regression is far larger
MIN_MID_FRAME = 5       # of the 19 saved frames, v8 put 7 in the reach band. Below 5 is a regression.


@pytest.mark.parametrize("band", ["near", "mid", "far"])
def test_band_has_labelled_fixtures(band):
    """Coverage, stated out loud. An empty band means this suite says NOTHING about the case the
    retrain is meant to fix, and that must be visible rather than green."""
    got = _frames_in(band)
    if not got:
        pytest.skip("no labelled frames for the %r band yet — see captures/qhm999rl-2026-08-19/"
                    % band)
    assert got


@pytest.mark.parametrize("band", ["near", "mid", "far"])
def test_emitter_is_detected_in_this_band(band):
    """The requirement, per band. `mid` is the one that matters: the robot needs reaction distance,
    and a model that only fires at y>=190 gives it a few centimetres."""
    detect = _ml()
    import cv2
    got = _frames_in(band)
    if not got:
        pytest.skip("no labelled frames for the %r band yet" % band)

    missed, weak = [], []
    for name, _note in got:
        p = os.path.join(FRAMES, name)
        im = cv2.imread(p)
        assert im is not None, "could not decode %s" % p
        r = detect(im)
        if not r.get("detected"):
            missed.append(name)
        elif float(r.get("confidence") or 0.0) < MIN_CONF:
            weak.append((name, r["confidence"]))

    msg = "%s band: %d/%d missed %s, %d below %.2f %s" % (
        band, len(missed), len(got), missed, len(weak), MIN_CONF, weak)
    if band in ("mid", "far") and (missed or weak):
        pytest.xfail("KNOWN, and the reason for the retrain — " + msg)
    assert not missed and not weak, msg


# ---------------------------------------------------------------- barebones, to fill in

@pytest.mark.skip(reason="TODO: needs the retrained model bound. Then this replaces the y-band "
                        "xfails as a hard requirement: recall over a whole lateral, counted "
                        "against the emitters physically on it.")
def test_recall_over_a_full_lateral_after_retraining():
    """N emitters on the tube, N stops. The field number to beat: 2-3 found out of ~12."""


@pytest.mark.skip(reason="TODO: needs plain-tube frames labelled as negatives. The 372-frame "
                        "stretch where the model returned nothing is the raw material, and it is "
                        "the control that says mid-confidence boxes elsewhere are not noise.")
def test_no_emitter_is_reported_on_plain_tube():
    """The false-positive half. Lowering _EMIT_CONF to 0.60 is only safe while this holds."""


# ---------------------------------------------------------------------------------------------
# The retrain, pinned. These run only on the board (the ML runner lives there).
# ---------------------------------------------------------------------------------------------

def test_retrained_model_still_detects_every_baseline_frame():
    """Every frame v8 detects must keep being detected, at close to the recorded confidence.

    Three of these were invisible to v6 and are the whole point of the retrain — losing them again
    is the regression this test exists to catch.
    """
    detect = _ml()
    import cv2
    missed, drifted = [], []
    for name, expect in sorted(BASELINE_V8.items()):
        p = os.path.join(FRAMES, name)
        if not os.path.exists(p):
            pytest.skip("baseline fixture missing: %s" % name)
        r = detect(cv2.imread(p))
        if not r.get("detected"):
            missed.append(name)
            continue
        got = float(r["confidence"])
        if abs(got - expect) > BASELINE_TOL:
            drifted.append((name, expect, round(got, 2)))
    assert not missed, "the retrained model stopped detecting: %s" % missed
    assert not drifted, "confidence moved more than %.2f from the v8 baseline: %s" % (
        BASELINE_TOL, drifted)


def test_mid_frame_recall_has_not_collapsed_back():
    """THE POINT OF THE RETRAIN. v6 fired only at y >= 190, leaving no reaction distance; v8 puts 7
    of 19 detections in the 130-190 reach band. If this falls back the robot is again seeing emitters
    too late to stop on them, and the confidence numbers alone would not show it."""
    detect = _ml()
    import cv2
    mid = 0
    frames = sorted(f for f in os.listdir(FRAMES) if f.endswith(".jpg"))
    for f in frames:
        r = detect(cv2.imread(os.path.join(FRAMES, f)))
        if r.get("detected") and 130 <= r["position"][1] < 190:
            mid += 1
    assert mid >= MIN_MID_FRAME, (
        "only %d of %d frames detected in the mid-frame reach band (want >= %d). v8 measured 7."
        % (mid, len(frames), MIN_MID_FRAME))


def test_the_retrained_model_does_not_fire_on_frames_with_no_emitter():
    """Recall bought with false positives is not recall. Measured: 0 boxes on 17 such frames.

    Uses the crossing negatives (bare soil with a lateral running the wrong way) and the align frames,
    four of which contain no tube at all. Neither set has anything to plant on.
    """
    detect = _ml()
    import cv2
    fired = []
    for sub in ("negative", "align"):
        d = os.path.join(os.path.dirname(FRAMES), sub)
        if not os.path.isdir(d):
            continue
        for f in sorted(os.listdir(d)):
            if not f.endswith((".jpg", ".png")):
                continue
            r = detect(cv2.imread(os.path.join(d, f)))
            if r.get("detected"):
                fired.append((sub + "/" + f, round(float(r["confidence"]), 2)))
    assert not fired, "fired on frames with no emitter: %s" % fired
