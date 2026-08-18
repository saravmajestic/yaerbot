"""Run the emitter model on its OWN thread, off the FrameBus.

THE PROBLEM THIS REPLACES. Inference costs 70-90ms and used to run inline in the control
loop, so the loop ticked at ~10Hz and every steering correction acted on a stale view. That
was patched with a 0.30s cache (`_EMIT_TTL_S`): run the model periodically, reuse the last
answer in between. It raised the loop rate, and it introduced a quieter fault — the cached
box's ROW sets the creep distance, so a 0.3s-old box places the seed up to 5cm from the
emitter, and seed placement is one of the few decisions here that no later frame can correct.

The cache was the wrong shape for the job, in three ways:

  * IT BLOCKED. Every TTL expiry, the steering loop stopped for 80ms to run a model that
    steering does not use. At 30fps that is two or three frames of the tube not being tracked,
    repeatedly, forever.
  * IT DID NOT KNOW HOW OLD IT WAS. The cache recorded when inference STARTED, which says
    nothing about the age of the world in the picture. Age is a property of the frame.
  * IT REUSED AN OLD ANSWER RATHER THAN COMPUTING A NEW ONE. Between expiries there were 9
    fresher frames sitting on the bus, unexamined.

This is the same problem FrameBus already solved for the camera, so it gets the same shape: a
thread that owns the slow thing, publishing only its LATEST result together with the capture
time of the frame it was computed from. The control loop reads that in nanoseconds and knows
exactly how stale it is, in seconds it can convert to centimetres of travel.

Deliberately knows nothing about Arduino, the console or the emitter model: it takes a bus and
a `detect(frame, moisture=...)` callable, so it is unit-testable off-device with stubs — which
matters, because the failure mode it exists to prevent is invisible until seed lands in the
wrong place.
"""

import threading
import time


class EmitterWorker:
    """Latest emitter detection, computed continuously off the control loop's critical path.

      worker = EmitterWorker(bus, detect_fn, moisture_fn).start()
      frame_t, result = worker.latest()      # frame_t = CAPTURE time, may be 0.0
      ...
      worker.stop()

    `min_interval` throttles how hard the accelerator is worked, NOT how stale an answer may
    be — those were the same number under the old cache and they are not the same concept.
    Left at 0 the worker runs flat out, which on this board is a continuously busy inference
    container for a result the robot only needs while an emitter is in the reach band.

    Sizing it: the reach band is the bottom 45% of the frame, about 108px, which at the
    measured 1090 px/m is ~10cm of ground and ~0.6s of travel at 0.170 m/s. 0.15s gives four
    independent looks inside that window, which is what the plant trigger needs, at roughly
    half the duty of running flat out. Unlike the old TTL, the control loop does not pay for
    this interval — it never waits on inference at all.
    """

    def __init__(self, bus, detect, moisture=None, min_interval=0.0, on_log=None):
        self._bus = bus
        self._detect = detect
        self._moisture = moisture or (lambda: None)
        self.min_interval = float(min_interval)
        self._log = on_log or (lambda m: None)
        self._lock = threading.Lock()
        self._t = 0.0                 # CAPTURE time of the frame the result came from
        self._res = None
        self._stop = threading.Event()
        self._thread = None
        self.runs = 0                 # inferences completed
        self.errors = 0
        self.skipped = 0              # frames passed over because no new one had arrived
        self._latency = None          # EMA of inference cost, seconds

    # ── lifecycle ──────────────────────────────────────────────────────────
    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def stop(self, timeout=2.0):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def alive(self):
        return self._thread is not None and self._thread.is_alive()

    # ── the worker ─────────────────────────────────────────────────────────
    def _run(self):
        last_frame_t = 0.0
        last_run = 0.0
        while not self._stop.is_set():
            now = time.time()
            if self.min_interval and (now - last_run) < self.min_interval:
                self._stop.wait(0.005)
                continue
            try:
                frame_t, frame = self._bus.latest()
            except Exception as e:                       # noqa: BLE001
                self.errors += 1
                self._log("emitter worker: bus.latest() raised: %s" % e)
                self._stop.wait(0.05)
                continue
            # ONLY EVER WORK ON A FRAME WE HAVE NOT SEEN. Re-running the model on the same
            # pixels burns the accelerator to produce an answer we already published, and it
            # would make `latest()` look fresher than the world it describes.
            if frame is None or frame_t <= last_frame_t:
                self.skipped += 1
                self._stop.wait(0.005)
                continue
            t0 = time.time()
            try:
                res = self._detect(frame, moisture=self._moisture())
            except Exception as e:                       # noqa: BLE001
                self.errors += 1
                self._log("emitter worker: detect raised: %s" % e)
                self._stop.wait(0.05)
                continue
            dt = time.time() - t0
            self._latency = dt if self._latency is None else 0.8 * self._latency + 0.2 * dt
            # Publish under the FRAME's capture time, not the completion time. The consumer is
            # asking "how old is the world this box describes", and inference latency is part
            # of that answer — charging it to the frame is what makes the number honest.
            with self._lock:
                self._t, self._res = frame_t, res
            last_frame_t, last_run = frame_t, time.time()
            self.runs += 1

    # ── consumers ──────────────────────────────────────────────────────────
    def latest(self):
        """(frame_capture_time, result). (0.0, None) before the first inference completes."""
        with self._lock:
            return self._t, self._res

    def age(self, now=None):
        """Seconds since the frame this result was computed from was captured."""
        with self._lock:
            if self._t == 0.0:
                return float("inf")
            return (now or time.time()) - self._t

    def latency(self):
        """Measured seconds per inference, or None before the first one."""
        return self._latency

    def stats(self):
        return {"runs": self.runs, "errors": self.errors, "skipped": self.skipped,
                "latency_ms": None if self._latency is None else round(self._latency * 1000),
                "alive": self.alive()}
