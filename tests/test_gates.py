"""Frame-rate-independent gates — the real functions from main.py, not a mirror of them.

WHY THIS FILE EXISTS, twice over.

1. Every gate in the camera loop used to be a number of seconds or pixels chosen against a
   camera that delivered a frame every ~710ms. The USB camera delivers one every ~33ms, so
   each of them changed meaning by 21x and NOTHING FAILED — the jump gate went inert, the
   traverse latch weakened by 20x, and the grace window became un-expirable. These tests pin
   the behaviour at BOTH frame rates so that can never be silent again.

2. The previous grace tests re-implemented the logic locally and asserted against the copy.
   They passed while the real function's signature changed underneath them. Everything here
   imports main.py through the stub-board harness and calls the real thing.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from test_console_imports import _load_main            # noqa: E402  (shared stub harness)

FAST = 0.0335        # USB webcam, measured 29.8 fps
SLOW = 0.710         # ESP32-CAM, measured 1.4 fps


def _tube(found=True, x=160.0, corr=1.0, far=1.5):
    return {"found": found, "tube_x": x, "x_near": x, "correction": corr,
            "far_correction": far, "width": 58, "strength": 3.6, "angle_deg": 0.0,
            "offset_px": x - 160.0}


# ── _gates: the derivation itself ─────────────────────────────────────────────

def test_gates_scale_with_the_measured_frame_interval():
    m = _load_main()
    fast, slow = m._gates(FAST), m._gates(SLOW)
    assert fast["jump_px"] < slow["jump_px"], \
        "a shorter frame interval must permit a SMALLER jump — that is the whole point"
    assert fast["grace_s"] < slow["grace_s"] or fast["grace_s"] == slow["grace_s"]


def test_jump_gate_is_bounded_at_both_ends():
    m = _load_main()
    # fast camera: bounded below by the DETECTOR's own noise (p99 = 18.8px, max 22.0px),
    # because gating tighter than the measurement noise throws away good readings
    assert m._gates(FAST)["jump_px"] >= 22.0
    assert m._gates(0.001)["jump_px"] == m._TRACK_JUMP_PX_MIN
    # slow camera: bounded above, so the gate never becomes a formality
    assert m._gates(SLOW)["jump_px"] == m._TRACK_JUMP_PX_MAX
    assert m._gates(None)["jump_px"] > 0, "must be usable before two frames have arrived"


def test_jump_gate_at_30fps_is_far_tighter_than_the_old_constant():
    """The regression this fixes: 70px was sized for 12-24cm of travel per frame."""
    m = _load_main()
    assert m._gates(FAST)["jump_px"] < 70, \
        "at 0.57cm of travel per frame a 70px jump gate rejects nothing"


# ── _track_tube: the jump gate in use ────────────────────────────────────────

def test_track_rejects_a_jump_that_the_old_gate_would_have_accepted():
    m = _load_main()
    m._track_reset()
    g = m._gates(FAST)
    now = 1000.0
    assert m._track_tube(_tube(x=160.0), 320, now, g["jump_px"])["found"] is True
    # 60px: comfortably inside the old 70px gate, well outside the derived one
    out = m._track_tube(_tube(x=220.0), 320, now + FAST, g["jump_px"])
    assert out["found"] is False
    assert out["rejected_x"] == 220.0


def test_track_still_accepts_normal_drift():
    """Do not break what worked: real per-frame drift is ~6px and must pass untouched."""
    m = _load_main()
    m._track_reset()
    g = m._gates(FAST)
    now, x = 1000.0, 160.0
    for i in range(20):
        x += 6.0
        out = m._track_tube(_tube(x=x), 320, now + i * FAST, g["jump_px"])
        assert out["found"] is True, "6px of drift per frame is the real signal"


def test_track_relocks_after_enough_rejections():
    """A genuine re-acquisition must still work, or losing the tube would be permanent."""
    m = _load_main()
    m._track_reset()
    g = m._gates(FAST)
    now = 1000.0
    m._track_tube(_tube(x=100.0), 320, now, g["jump_px"])
    accepted = False
    for i in range(1, m._TRACK_RELOCK_N + 3):
        out = m._track_tube(_tube(x=280.0), 320, now + i * FAST, g["jump_px"])
        accepted = accepted or out["found"]
    assert accepted, "after _TRACK_RELOCK_N rejects the new position must be believed"


# ── grace: distance budget, miss cap, and decay ──────────────────────────────

def test_a_single_missed_frame_does_not_stop_the_robot():
    """The reported symptom: robot moved 10cm then stopped. It had not lost the tube — it
    hit one bad frame in three at a 27% miss rate."""
    m = _load_main()
    m._tube_grace_reset()
    g = m._gates(FAST)
    now = 1000.0
    seq = [True, False, True, False, False, True]
    held = 0
    for i, found in enumerate(seq):
        out, holding = m._tube_with_grace(_tube(found=found), now + i * FAST, g)
        assert out["found"] is True, "a brief gap must not stop the motors"
        held += 1 if holding else 0
    assert held == 3


def test_a_real_loss_still_stops_it_and_within_a_bounded_distance():
    m = _load_main()
    m._tube_grace_reset()
    g = m._gates(FAST)
    now = 1000.0
    m._tube_with_grace(_tube(found=True), now, g)
    lost_at = None
    for i in range(1, 60):
        out, _ = m._tube_with_grace(_tube(found=False), now + i * FAST, g)
        if not out["found"]:
            lost_at = i
            break
    assert lost_at is not None, "grace must expire — it is a tolerance, not a blindfold"
    blind_m = lost_at * FAST * m._DRIP_SPEED_MPS
    assert blind_m < 0.05, "must not drive more than ~5cm blind, was 6.8cm at 0.4s"
    assert lost_at <= m._TUBE_GRACE_MAX_MISSES + 1


def test_at_30fps_the_two_bounds_very_nearly_coincide():
    """Not an accident, and worth pinning: 0.035m at 0.170 m/s is 0.206s, and 6 frames at
    33.5ms is 0.201s. Either bound expires the grace at the same frame, so at THIS frame rate
    the miss cap is redundant. Mutation testing caught an earlier version of this file
    crediting the cap for work the distance budget was doing."""
    m = _load_main()
    g = m._gates(FAST)
    by_distance = g["grace_s"] / FAST
    assert abs(by_distance - m._TUBE_GRACE_MAX_MISSES) < 1.5, \
        "bounds have drifted apart: %.1f frames by distance vs %d by cap" \
        % (by_distance, m._TUBE_GRACE_MAX_MISSES)


def test_the_miss_cap_binds_on_a_faster_camera():
    """Where the cap earns its keep. At 60fps the distance budget is ~12 frames, so without
    the cap the robot would carry a dead reading for twice as many frames. This is the test
    that fails if the cap is removed — the 30fps one cannot see it."""
    m = _load_main()
    very_fast = 1.0 / 60
    g = m._gates(very_fast)
    assert g["grace_s"] / very_fast > m._TUBE_GRACE_MAX_MISSES + 3, \
        "sanity: at 60fps the distance budget must be the looser of the two"
    m._tube_grace_reset()
    now = 1000.0
    m._tube_with_grace(_tube(found=True), now, g)
    for i in range(1, m._TUBE_GRACE_MAX_MISSES + 1):
        out, holding = m._tube_with_grace(_tube(found=False), now + i * very_fast, g)
        assert out["found"] is True and holding, "frame %d: within the cap it must drive" % i
    out, _ = m._tube_with_grace(_tube(found=False),
                                now + (m._TUBE_GRACE_MAX_MISSES + 1) * very_fast, g)
    assert out["found"] is False, "the miss cap must expire the grace before the distance does"


def test_slow_camera_still_gets_at_least_one_frame_of_grace():
    """The distance budget alone is shorter than one ESP32-CAM frame, which would make the
    grace useless exactly on the camera that drops the most frames."""
    m = _load_main()
    m._tube_grace_reset()
    g = m._gates(SLOW)
    assert g["grace_s"] >= SLOW, "grace must outlast one frame interval"


def test_on_a_slow_camera_the_distance_budget_binds_not_the_frame_count():
    """Where the distance budget earns its keep, and why a frame count alone is not enough.
    Six missed frames at 1.4fps is 4.3 SECONDS — around 72cm of blind driving. Time-based
    thinking hides that completely; the bound has to be in metres."""
    m = _load_main()
    g = m._gates(SLOW)
    m._tube_grace_reset()
    now = 1000.0
    m._tube_with_grace(_tube(found=True), now, g)
    lost_at = None
    for i in range(1, 20):
        out, _ = m._tube_with_grace(_tube(found=False), now + i * SLOW, g)
        if not out["found"]:
            lost_at = i
            break
    assert lost_at is not None
    assert lost_at < m._TUBE_GRACE_MAX_MISSES, \
        "the distance budget must expire first on a slow camera, not the %d-frame cap" \
        % m._TUBE_GRACE_MAX_MISSES
    blind_m = lost_at * SLOW * m._DRIP_SPEED_MPS
    assert blind_m < 0.25, "blind distance %.2fm is too far to drive on a dead reading" % blind_m


def test_the_held_correction_decays_instead_of_being_replayed():
    """Replaying the last command unchanged is a 0.4s open-loop turn: the longer the robot is
    blind the harder it commits. It must straighten instead."""
    m = _load_main()
    m._tube_grace_reset()
    g = m._gates(FAST)
    now = 1000.0
    m._tube_with_grace(_tube(found=True, corr=2.0, far=3.0), now, g)
    corrs = []
    for i in range(1, m._TUBE_GRACE_MAX_MISSES + 1):
        out, holding = m._tube_with_grace(_tube(found=False), now + i * FAST, g)
        if holding:
            corrs.append(abs(out["correction"]))
    assert len(corrs) >= 3
    assert corrs == sorted(corrs, reverse=True), "must decay monotonically, got %r" % corrs
    assert corrs[0] < 2.0, "even the first held frame is already stale"
    assert corrs[-1] < corrs[0] * 0.6, "must be substantially wound down by the end"


def test_grace_cannot_start_without_ever_seeing_the_tube():
    """No held reading exists at the start of a run, so it must not drive on nothing."""
    m = _load_main()
    m._tube_grace_reset()
    g = m._gates(FAST)
    for i in range(5):
        out, holding = m._tube_with_grace(_tube(found=False), 1000.0 + i * FAST, g)
        assert out["found"] is False and holding is False


def test_a_fresh_reading_resets_the_miss_count():
    m = _load_main()
    m._tube_grace_reset()
    g = m._gates(FAST)
    now = 1000.0
    for cycle in range(4):                       # miss a few, then see it, repeatedly
        m._tube_with_grace(_tube(found=True), now, g)
        now += FAST
        for _ in range(m._TUBE_GRACE_MAX_MISSES - 2):
            out, _ = m._tube_with_grace(_tube(found=False), now, g)
            now += FAST
            assert out["found"] is True, "cycle %d: intermittent detection must keep driving" % cycle


# ── the traverse latch: what decides the robot turns onto a row ──────────────

def _run_traverse(m, samples, step_m=0.0057):
    """Feed (found, tube_y) samples spaced step_m apart. Returns the last info dict."""
    hist, info, travelled = [], None, 0.0
    for found, y in samples:
        travelled += step_m
        hist, info = m._traverse_track(hist, travelled,
                                       {"found": found, "tube_y": y})
    return info


def test_a_static_artefact_seen_in_every_frame_does_not_latch():
    """The failure mode the old rule allowed: something detected constantly but not
    approaching. A shadow or a straw sits still; a lateral we are driving at does not."""
    m = _load_main()
    info = _run_traverse(m, [(True, 120.0)] * 60)
    assert info["sights"] == info["frames"], "sanity: it was seen every frame"
    assert info["approaching"] is False, "constant detection is not approach"


def test_a_slowly_drifting_artefact_does_not_latch():
    """Noise gives a band a few px of wander. Over 42cm of driving the old rule needed only
    25px of growth, which wander supplies for free."""
    m = _load_main()
    samples = [(True, 120.0 + (i % 5) * 4.0) for i in range(60)]
    info = _run_traverse(m, samples)
    assert info["approaching"] is False


def test_a_real_approaching_lateral_latches():
    """Geometry: a lateral sweeps down the frame at _PX_PER_M_DEPTH px per metre driven."""
    m = _load_main()
    step = 0.0057
    samples = [(True, 10.0 + i * step * m._PX_PER_M_DEPTH) for i in range(24)]
    info = _run_traverse(m, samples, step_m=step)
    assert info["approaching"] is True, info


def test_dropouts_thin_the_history_but_do_not_reset_it():
    """The 03:33 run never turned because one dropped frame reset a consecutive counter."""
    m = _load_main()
    step = 0.0057
    samples = []
    for i in range(30):
        y = 10.0 + i * step * m._PX_PER_M_DEPTH
        samples.append((i % 3 != 0, y))          # lose every third frame
    info = _run_traverse(m, samples, step_m=step)
    assert info["approaching"] is True, info
    assert info["frac"] < 1.0, "sanity: frames really were dropped"


def test_a_sparse_hit_rate_does_not_latch_at_30fps():
    """The actual regression: 4 sightings in 75 frames used to pass. It must not."""
    m = _load_main()
    step = 0.0057
    samples = []
    for i in range(75):
        y = 10.0 + i * step * m._PX_PER_M_DEPTH
        samples.append((i % 19 == 0, y))         # ~5% hit rate, and genuinely approaching
    info = _run_traverse(m, samples, step_m=step)
    assert info["frac"] < m._TRAVERSE_MIN_SIGHT_FRAC
    assert info["approaching"] is False, \
        "a 5%% hit rate is not evidence, however well the few sightings line up"


def test_the_growth_requirement_grows_with_the_sighting_span():
    """Capping the requirement would make a longer observation an EASIER test."""
    m = _load_main()
    short = _run_traverse(m, [(True, 10.0 + i * 2.0) for i in range(5)], step_m=0.0057)
    longr = _run_traverse(m, [(True, 10.0 + i * 2.0) for i in range(40)], step_m=0.0057)
    assert longr["need_px"] > short["need_px"]


def test_the_capture_interval_can_be_set_without_restarting_capture():
    """A control that silently does nothing is worse than a missing control.

    The interval used to be read in exactly ONE place (capture_start), so every other route
    into capture used whatever was left in _capture. Scan mode turns capture on by itself
    without consulting the slider, so moving the slider to 0.5s and pressing "Follow tube &
    capture" saved 127 frames at the 2.0s default — 34cm apart, stepping over most emitters,
    which was the exact problem the 0.5s was meant to solve.
    """
    m = _load_main()
    h = m.ui.handlers
    assert "capture_interval" in h, "the handler must be registered or the UI cannot reach it"

    h["capture_start"](None, {"interval": 2.0})
    assert m._capture["on"] is True and m._capture["interval"] == 2.0

    h["capture_interval"](None, {"interval": 0.5})          # while running
    assert m._capture["interval"] == 0.5, "must apply live, not on the next start"

    h["capture_stop"](None, {})
    h["capture_interval"](None, {"interval": 0.0})          # while stopped
    assert m._capture["interval"] == 0.0, "must be settable before capture starts too"

    # and scan mode must then inherit it rather than resetting to a default
    m._capture["on"] = False
    m._plot["corners"] = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    m._cam["url"] = "usb"
    m._VISION_OK = True
    h["run_start"](None, {"mode": "scan"})
    assert m._capture["interval"] == 0.0, \
        "scan-mode auto-capture must use the operator's interval, not overwrite it"
    h["run_stop"](None, {})


def test_dense_capture_does_not_flood_the_operator_socket():
    """Dataset capture can now run at the full frame rate (interval 0), which is what makes a
    consecutive-frame set possible at all. But _save_capture used to push a base64 thumbnail
    on EVERY save, and 30 of those a second would stall the control loop inside send_message —
    the loop that is steering the robot. Saving is the point; the preview is reassurance.
    """
    import numpy as np
    m = _load_main()
    sent = []
    real = m.ui.send_message
    m.ui.send_message = lambda name, payload=None: sent.append(name)
    try:
        m._cap_thumb_last[0] = 0.0
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        before = m._capture["count"]
        for _ in range(30):                      # one second of frames at 30fps
            m._save_capture(frame)
    finally:
        m.ui.send_message = real
    assert m._capture["count"] - before == 30, "every frame must still be SAVED"
    thumbs = [s for s in sent if s == "capture_saved"]
    assert len(thumbs) <= 3, \
        "pushed %d thumbnails for 30 saves — must be throttled" % len(thumbs)
    assert len(thumbs) >= 1, "the operator still needs to see frames arriving"


def test_the_window_cannot_be_narrowed_into_latching_on_noise():
    """Which direction of this constant is actually dangerous.

    Widening _TRAVERSE_WINDOW_M is self-limiting: more frames in the window means more misses
    diluting the sighting fraction, so it gets HARDER to latch. Mutation testing confirmed a
    99m window changes no verdict. Narrowing it is the risk — with only a few frames in scope
    the sighting span collapses, need_px falls to its absolute floor, and a couple of px of
    sensor noise clears it. So the floor must be large enough that a short span cannot pass on
    noise alone.
    """
    m = _load_main()
    # three sightings inside 1cm of driving: a real lateral moves 11px in that distance, so
    # nothing measured over such a span should ever be trusted as evidence of approach
    info = _run_traverse(m, [(True, 120.0), (True, 123.0), (True, 126.0)], step_m=0.003)
    assert info["approaching"] is False, \
        "a 6px wobble over 1cm of driving is not an approach (need_px=%.1f, grew=%.1f)" \
        % (info["need_px"], info["grew"])


def test_something_moving_far_slower_than_the_geometry_does_not_latch():
    """THE CAP BUG, pinned. An earlier draft of _traverse_track wrote

        need_px = min(25.0, max(FLOOR, FRAC * span_m * PX_PER_M))

    which caps the requirement at 25px. That cap is invisible to a short-span test (where the
    requirement is already below 25) and it silently restores the old, far-too-weak rule over
    long spans. Mutation testing showed the rest of this file could not see it.

    Here a band drifts 40px while the robot drives 22cm. A real lateral would have swept ~240px
    in that distance, so 40px is a sixth of the required rate and must be rejected — but 40px
    clears a 25px cap easily.
    """
    m = _load_main()
    step = 0.0057
    n = 40
    total_growth = 40.0
    samples = [(True, 10.0 + i * (total_growth / (n - 1))) for i in range(n)]
    info = _run_traverse(m, samples, step_m=step)
    assert info["frac"] == 1.0 and info["sights"] == n, "sanity: seen every frame"
    assert info["grew"] > 25.0, "sanity: it grew more than the old absolute floor"
    assert info["need_px"] > info["grew"], \
        "the geometric requirement (%.0fpx over %.2fm) must exceed the drift (%.0fpx)" \
        % (info["need_px"], info["span_m"], info["grew"])
    assert info["approaching"] is False, \
        "drifting at a sixth of the geometric rate is not an approach"
