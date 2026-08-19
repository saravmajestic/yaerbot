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
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "console", "python"))

FRAMES = os.path.join(_HERE, "frames", "emitter")


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
