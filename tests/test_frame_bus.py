"""FrameBus — the camera reader thread, and what it guarantees.

The control loop blocks for seconds at a time (2s of plant dwell per emitter, ~1.5s per
pivot, ~1.7s creeping onto a lateral). Two things used to go wrong in that window and both
are silent, which is what makes them worth pinning with tests:

  * nobody read the socket, so an MJPEG camera that could not complete a send abandoned the
    frame ("Stream ends prematurely at 5675" in the field log) and the stream was reopened;
  * the next read returned a BUFFERED frame from before the block, with no way to tell —
    so the robot would steer on where the tube used to be.
"""
import os
import sys
import time

import pytest

cv2 = pytest.importorskip("cv2", reason="OpenCV not installed off-device")
np = pytest.importorskip("numpy")

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "console", "python"))

from vision.camera import FrameBus            # noqa: E402


@pytest.fixture
def still(tmp_path):
    """A still image is a legitimate FrameSource, and it delivers as fast as we read."""
    p = tmp_path / "soil.png"
    cv2.imwrite(str(p), np.full((240, 320, 3), 150, np.uint8))
    return str(p)


def test_publishes_frames_with_a_capture_time(still):
    bus = FrameBus(still).start()
    try:
        t, frame = bus.wait_fresh(timeout=2.0)
        assert frame is not None
        assert t > 0
        assert bus.age() < 0.5
    finally:
        bus.stop()


def test_a_blocking_caller_gets_a_FRESH_frame_afterwards(still):
    """THE point of the whole class. Sleep like a plant dwell, then demand a frame captured
    after the sleep ended — the reader kept working throughout, so one exists."""
    bus = FrameBus(still).start()
    try:
        bus.wait_fresh(timeout=2.0)
        blocked_until = time.time() + 1.0
        while time.time() < blocked_until:       # stand in for plantSeed / pivotDeg
            time.sleep(0.05)
        t, frame = bus.wait_fresh(after_t=blocked_until, timeout=2.0)
        assert frame is not None, "no frame captured after the block — the reader stalled"
        assert t > blocked_until
        assert bus.age() < 0.5, "frame is stale: the reader is not keeping up"
    finally:
        bus.stop()


def test_keeps_only_the_newest_frame(still):
    """A queue would preserve exactly the staleness this exists to remove: after idling,
    the frame on offer must be recent, not the first of a backlog."""
    bus = FrameBus(still).start()
    try:
        bus.wait_fresh(timeout=2.0)
        first_t, _ = bus.latest()
        time.sleep(0.6)                          # a backlog would build up here
        later_t, _ = bus.latest()
        assert later_t > first_t                 # it moved on
        assert bus.age() < 0.5                   # ...to something current
    finally:
        bus.stop()


def test_age_reports_infinity_before_the_first_frame(still):
    """Callers gate on age, so 'no frame yet' must not read as 'age zero' — that would
    look like a perfectly fresh frame and get acted on."""
    bus = FrameBus(still)
    assert bus.age() == float("inf")
    t, frame = bus.latest()
    assert frame is None and t == 0.0


def test_wait_fresh_times_out_rather_than_hanging(still):
    """A dead camera must not wedge the control loop — the caller needs the None back so
    it can stop the robot."""
    bus = FrameBus(still).start()
    try:
        bus.wait_fresh(timeout=2.0)
        future = time.time() + 30                # nothing can satisfy this
        t0 = time.time()
        t, frame = bus.wait_fresh(after_t=future, timeout=0.3)
        assert frame is None and t == 0.0
        assert time.time() - t0 < 1.0
    finally:
        bus.stop()


def test_stop_is_idempotent_and_ends_the_thread(still):
    bus = FrameBus(still).start()
    bus.wait_fresh(timeout=2.0)
    bus.stop()
    assert not bus.alive()
    bus.stop()                                   # must not raise on a second call


# ── MjpegStream: the validating decoder ───────────────────────────────────────
# The camera genuinely emits incomplete JPEGs (esp32-camera #417/#252: the driver can
# overwrite the buffer mid-send, and ~5% of frames lack the end-of-image marker). FFmpeg
# accepts those and renders the missing blocks green; OpenCV's own decoder refuses them.
# These tests pin the refusal, because it is what keeps a corrupt frame off the robot
# without needing the camera reflashed.

def _multipart(jpegs, boundary=b"123456789000000000000987654321"):
    """A stream shaped exactly like the cam's: boundary, Content-Length, then bytes."""
    out = b""
    for j in jpegs:
        out += b"--" + boundary + b"\r\n"
        out += b"Content-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(j)
        out += j + b"\r\n"
    return out


def _jpeg_bytes(tmp_path, value=150):
    p = tmp_path / ("f%d.jpg" % value)
    cv2.imwrite(str(p), np.full((240, 320, 3), value, np.uint8))
    return open(str(p), "rb").read()


def test_parser_extracts_each_frame(tmp_path):
    import io
    from vision.camera import _next_jpeg
    a, b = _jpeg_bytes(tmp_path, 100), _jpeg_bytes(tmp_path, 200)
    fp = io.BytesIO(_multipart([a, b]))
    bound = b"123456789000000000000987654321"
    assert _next_jpeg(fp, bound) == a
    assert _next_jpeg(fp, bound) == b
    assert _next_jpeg(fp, bound) is None          # end of stream


def test_a_truncated_frame_is_REJECTED_not_rendered_green(tmp_path):
    """The whole reason this class exists. FFmpeg returns this frame 58% green; a
    validating decoder returns nothing at all, so it never reaches the detector."""
    import io
    from vision.camera import _next_jpeg
    good = _jpeg_bytes(tmp_path, 150)
    cut = good[:int(len(good) * 0.45)]            # abandoned mid-send, as the camera does
    fp = io.BytesIO(_multipart([cut, good]))
    bound = b"123456789000000000000987654321"

    raw = _next_jpeg(fp, bound)
    assert raw == cut                            # the parser hands up what arrived
    assert cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR) is None, \
        "a truncated JPEG must not decode — that is what produces the green block"

    raw = _next_jpeg(fp, bound)
    assert cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR) is not None


def test_parser_survives_a_stream_cut_mid_headers(tmp_path):
    """The stream dies at arbitrary points ('Stream ends prematurely at 5675'), and a
    half-written header must return None rather than raise into the reader thread."""
    import io
    from vision.camera import _next_jpeg
    bound = b"123456789000000000000987654321"
    assert _next_jpeg(io.BytesIO(b"--" + bound + b"\r\nContent-Len"), bound) is None
    assert _next_jpeg(io.BytesIO(b""), bound) is None
    # length declared but the body never arrives
    fp = io.BytesIO(b"--" + bound + b"\r\nContent-Length: 5000\r\n\r\n" + b"\x00" * 10)
    assert _next_jpeg(fp, bound) is None


def test_stale_limit_MEASURES_the_camera_rather_than_assuming(still):
    """A fixed 0.25s limit was catastrophic on a camera delivering ~1.4fps: every frame
    was already 'too old', so the loop stopped the motors between frames and the robot
    spluttered instead of driving. The limit must track the observed frame interval."""
    bus = FrameBus(still).start()
    try:
        bus.wait_fresh(timeout=2.0)
        time.sleep(0.5)                       # let a few intervals be measured
        iv = bus.interval()
        assert iv is not None and iv > 0, "interval should be measured once frames flow"
        limit = bus.stale_after()
        assert limit >= iv * 2, \
            "the limit must be comfortably above the frame interval, or normal " \
            "delivery reads as a dead stream"
        assert limit <= 3.0                   # but still bounded, so a dead stream is caught
    finally:
        bus.stop()


def test_stale_limit_is_generous_before_any_frame_arrives(still):
    """With no measurement yet, assume the worst rather than the best — otherwise the
    first frames of a run get rejected for being slower than a guess."""
    bus = FrameBus(still)
    assert bus.interval() is None
    assert bus.stale_after() == 3.0


# ── Frame size is PINNED ──────────────────────────────────────────────────────
# Every detector constant is in pixels at 320x240: _TUBE_MIN_W_PX, _profile_line's
# min_w/max_w, max_drift_px. The Logitech C310 ignores CAP_PROP_FRAME_WIDTH and delivers
# 640x480 regardless, which would double every measured width and get real tubes thrown out
# as implausible. So the size is enforced on every frame, not merely requested.

def test_frames_are_pinned_to_the_detector_resolution(tmp_path):
    from vision.camera import FrameSource, FRAME_W, FRAME_H
    big = tmp_path / "big.png"
    cv2.imwrite(str(big), np.full((480, 640, 3), 150, np.uint8))   # wrong size on purpose
    src = FrameSource(str(big))
    ok, frame = src.read()
    assert ok
    assert frame.shape[1] == FRAME_W and frame.shape[0] == FRAME_H, \
        "a frame at the wrong size must be scaled, or every pixel threshold is invalid"
    assert src.resized >= 1, "the rescale should be counted so the mismatch is visible"


def test_a_correctly_sized_frame_is_not_touched(tmp_path):
    from vision.camera import FrameSource, FRAME_W, FRAME_H
    right = tmp_path / "right.png"
    cv2.imwrite(str(right), np.full((FRAME_H, FRAME_W, 3), 150, np.uint8))
    src = FrameSource(str(right))
    ok, frame = src.read()
    assert ok and frame.shape[:2] == (FRAME_H, FRAME_W)
    assert src.resized == 0, "no resize should happen when the driver already agrees"


def test_usb_camera_sources_are_recognised():
    """The UI passes a string. A USB camera can arrive as '2' or '/dev/video2', and OpenCV
    wants the integer index for a V4L2 device — so the bus normalises it rather than making
    the operator know that."""
    import inspect
    from vision.camera import FrameBus
    src = inspect.getsource(FrameBus._open)
    assert "/dev/video" in src and "isdigit" in src and "http" in src


# ── Tube grace: one bad frame must not halt the robot ─────────────────────────
# The detector achieves 69% on real captures; the misses are frames where the robot's own
# shadow corrupts one band. At 30fps that is still ~20 good readings a second, but stopping
# on the FIRST miss made the robot halt after ~10cm of travel — three times a second.

def _grace(found_seq, grace_s=0.4, dt=0.033):
    """Replay a found/not-found sequence through the same logic main.py uses."""
    hold = {"t": 0.0, "tube": None}
    now = 1000.0
    out = []
    for found in found_seq:
        now += dt
        tube = {"found": found, "tube_x": 100.0}
        if tube["found"]:
            hold["t"], hold["tube"] = now, tube
            out.append(("fresh", True))
        elif hold["tube"] is not None and (now - hold["t"]) <= grace_s:
            out.append(("held", True))
        else:
            out.append(("lost", False))
    return out


def test_a_single_missed_frame_does_not_stop_the_robot():
    """The reported symptom: robot moved 10cm then stopped. It had not lost the tube — it
    hit one bad frame in three."""
    seq = _grace([True, False, True, False, False, True])
    assert all(driving for _, driving in seq), "a brief gap must not stop the motors"
    assert [k for k, _ in seq].count("held") == 3


def test_a_real_loss_still_stops_it():
    """Grace is a tolerance, not a blindfold — past the window the robot must stop, which is
    what protects it when the tube genuinely leaves the view."""
    seq = _grace([True] + [False] * 40)          # 40 frames at 33ms = 1.3s of nothing
    assert seq[0][1] is True
    assert seq[-1] == ("lost", False)
    held = [i for i, (k, _) in enumerate(seq) if k == "held"]
    assert len(held) <= 14, "grace should expire after ~0.4s, not carry indefinitely"


def test_grace_cannot_start_without_ever_seeing_the_tube():
    """No held reading exists at the start of a run, so it must not drive on nothing."""
    seq = _grace([False, False, False])
    assert all(not driving for _, driving in seq)
