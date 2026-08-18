"""EmitterWorker — inference off the control loop's critical path.

The claims being tested are concurrency claims, so they are tested against a deliberately SLOW
stub detector and a stub bus rather than the real model. The failure this guards against is
invisible in normal operation: a box computed from an old frame still looks like a box, and its
row is what sets the creep distance, so a wrong answer here plants seed in the wrong place and
nothing downstream can notice.
"""
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "console", "python"))

from vision.emitter_worker import EmitterWorker          # noqa: E402


class FakeBus:
    """Publishes frames on demand, mimicking FrameBus.latest()'s (capture_time, frame)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._t = 0.0
        self._frame = None
        self.reads = 0

    def publish(self, frame, t=None):
        with self._lock:
            self._t = time.time() if t is None else t
            self._frame = frame

    def latest(self):
        with self._lock:
            self.reads += 1
            return self._t, self._frame


def _slow_detector(cost=0.05, results=None):
    """A detector that takes real time, so 'does not block the caller' means something."""
    calls = []

    def detect(frame, moisture=None):
        calls.append((frame, moisture))
        time.sleep(cost)
        if results is not None:
            return results(frame)
        return {"detected": True, "position": (100, frame), "confidence": 0.9}

    detect.calls = calls
    return detect


def _wait(pred, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.005)
    return False


def test_the_result_carries_the_frames_capture_time_not_the_completion_time():
    """The whole reason this exists. The consumer asks "how old is the world this box
    describes", and inference latency is part of that answer — charging it to the frame is what
    makes the staleness number honest. The old cache stamped when inference STARTED, which says
    nothing about the age of the picture."""
    bus = FakeBus()
    det = _slow_detector(cost=0.08)
    w = EmitterWorker(bus, det).start()
    try:
        capture_t = time.time()
        bus.publish(7, t=capture_t)
        assert _wait(lambda: w.latest()[1] is not None)
        got_t, res = w.latest()
        assert got_t == capture_t, "must republish the frame's own capture time"
        assert res["position"] == (100, 7)
        # age includes the inference cost, because that time really did pass
        assert w.age() >= 0.08
    finally:
        w.stop()


def test_the_worker_does_not_block_the_caller():
    """Inference costs 70-90ms on the board. The control loop must not pay it."""
    bus = FakeBus()
    w = EmitterWorker(bus, _slow_detector(cost=0.10)).start()
    try:
        bus.publish(1)
        t0 = time.time()
        for _ in range(200):
            w.latest()
            w.age()
        elapsed = time.time() - t0
        assert elapsed < 0.05, \
            "200 reads took %.3fs — latest() must never wait on inference" % elapsed
    finally:
        w.stop()


def test_it_never_runs_twice_on_the_same_frame():
    """Re-running the model on identical pixels burns the accelerator to produce an answer we
    already published, and would make latest() look fresher than the world it describes."""
    bus = FakeBus()
    det = _slow_detector(cost=0.01)
    w = EmitterWorker(bus, det).start()
    try:
        bus.publish(42, t=1000.0)
        assert _wait(lambda: len(det.calls) >= 1)
        time.sleep(0.15)                     # plenty of chances to run again
        assert len(det.calls) == 1, "ran %d times on one frame" % len(det.calls)
        assert w.skipped > 0, "should have been actively skipping the stale frame"
    finally:
        w.stop()


def test_it_always_picks_up_the_newest_frame():
    """It pulls from the bus rather than being fed, so frames that arrive during an inference
    are not queued — the next run uses the LATEST, exactly like FrameBus itself."""
    bus = FakeBus()
    det = _slow_detector(cost=0.05)
    w = EmitterWorker(bus, det).start()
    try:
        for i in range(1, 8):
            bus.publish(i, t=1000.0 + i)
            time.sleep(0.01)                 # faster than inference
        assert _wait(lambda: w.latest()[0] >= 1005.0, timeout=2.0)
        seen = [c[0] for c in det.calls]
        assert len(seen) < 7, "must skip frames rather than queue them, saw %r" % seen
        assert max(seen) >= 5, "must reach the recent frames, saw %r" % seen
    finally:
        w.stop()


def test_min_interval_throttles_the_inference_rate():
    """It is a throttle on the accelerator, not a staleness budget — and unlike the old TTL the
    control loop does not pay for it."""
    bus = FakeBus()
    det = _slow_detector(cost=0.001)
    w = EmitterWorker(bus, det, min_interval=0.05).start()
    try:
        end = time.time() + 0.4
        i = 0
        while time.time() < end:
            i += 1
            bus.publish(i)
            time.sleep(0.005)                # ~80 frames offered in 0.4s
        w.stop()
        assert len(det.calls) <= 12, "throttle ignored: %d inferences in 0.4s" % len(det.calls)
        assert len(det.calls) >= 3, "throttle too aggressive: only %d" % len(det.calls)
    finally:
        w.stop()


def test_a_detector_exception_does_not_kill_the_thread():
    """A model that throws must degrade to 'no detection', not silently stop detecting for the
    rest of the run — which is what an unguarded thread would do."""
    bus = FakeBus()
    state = {"n": 0}

    def flaky(frame, moisture=None):
        state["n"] += 1
        if state["n"] <= 3:
            raise RuntimeError("inference blew up")
        return {"detected": True, "position": (1, 2), "confidence": 0.5}

    logs = []
    w = EmitterWorker(bus, flaky, on_log=logs.append).start()
    try:
        for i in range(1, 12):
            bus.publish(i, t=2000.0 + i)
            time.sleep(0.02)
        assert _wait(lambda: w.latest()[1] is not None, timeout=2.0), \
            "worker never recovered after the exceptions"
        assert w.errors >= 3
        assert w.alive(), "thread must survive a throwing detector"
        assert any("detect raised" in m for m in logs), "failures must be visible in the log"
    finally:
        w.stop()


def test_a_bus_exception_does_not_kill_the_thread():
    class BrokenBus:
        def __init__(self):
            self.n = 0

        def latest(self):
            self.n += 1
            if self.n <= 3:
                raise RuntimeError("bus gone")
            return 5000.0, 1

    logs = []
    w = EmitterWorker(BrokenBus(), _slow_detector(cost=0.001), on_log=logs.append).start()
    try:
        assert _wait(lambda: w.latest()[1] is not None, timeout=2.0)
        assert w.alive()
        assert any("bus.latest() raised" in m for m in logs)
    finally:
        w.stop()


def test_before_the_first_inference_it_reports_no_result_and_infinite_age():
    """The control loop must be able to tell "nothing yet" from "detected nothing"."""
    bus = FakeBus()
    w = EmitterWorker(bus, _slow_detector(cost=0.2))
    t, res = w.latest()
    assert t == 0.0 and res is None
    assert w.age() == float("inf")


def test_stop_is_idempotent_and_ends_the_thread():
    bus = FakeBus()
    bus.publish(1)
    w = EmitterWorker(bus, _slow_detector(cost=0.001)).start()
    w.stop()
    assert not w.alive()
    w.stop()                                  # must not raise
    assert not w.alive()


def test_moisture_is_read_per_inference_not_captured_once():
    """The probe reading must not be frozen at construction — it is a live sensor."""
    bus = FakeBus()
    vals = iter([11, 22, 33, 44, 55, 66, 77, 88])
    seen = []

    def detect(frame, moisture=None):
        seen.append(moisture)
        return {"detected": False, "position": None, "confidence": 0.0}

    w = EmitterWorker(bus, detect, moisture=lambda: next(vals, 99)).start()
    try:
        for i in range(1, 5):
            bus.publish(i, t=3000.0 + i)
            time.sleep(0.03)
        assert _wait(lambda: len(seen) >= 2, timeout=2.0)
        assert len(set(seen)) > 1, "moisture was read once and reused: %r" % seen
    finally:
        w.stop()
