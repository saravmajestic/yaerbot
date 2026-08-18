"""
Farm OS — Stage 5 camera source abstraction.

One interface over: the ESP32-CAM MJPEG stream (on-device), a video file, a
still image, or a local webcam — so the vision pipeline develops/tests OFFLINE
(file/image) and runs unchanged on the robot (ESP32-CAM URL).

  cam = FrameSource("http://192.168.4.50:81/stream")   # ESP32-CAM (on robot)
  cam = FrameSource("samples/driphose.mp4")            # video (offline dev)
  cam = FrameSource("samples/tube.png")                # still image (unit tests)
  cam = FrameSource(0)                                  # laptop webcam

  ok, frame = cam.read()
"""

import os
import re
import subprocess
import threading
import time
import urllib.request

import cv2
import numpy as np


# EVERY DETECTOR CONSTANT IS IN PIXELS AT THIS SIZE.
# _TUBE_MIN_W_PX, _profile_line's min_w/max_w, max_drift_px, the emitter reach fraction —
# all were measured against the ESP32-CAM's 320x240 frames. Running the pipeline at any
# other size silently invalidates all of them: a tube 38px wide at QVGA is 76px at VGA and
# would be thrown out as implausible. So the frame size is PINNED here, and enforced on every
# frame rather than requested and hoped for — not because the request fails, but because a
# camera that quietly ignores it must not be able to re-scale every constant in the pipeline.
#
# CORRECTION 2026-08-18: an earlier note here claimed the C310 IGNORES CAP_PROP_FRAME_WIDTH
# and always delivers 640x480. That was wrong, and it was measured on a bare probe that never
# set the property. 320x240 YUYV @30fps is a native C310 mode and the request DOES take
# effect, so _fit() is a no-op in this deployment:
#     v4l2-ctl --device=/dev/video0 --get-fmt-video   ->  Width/Height : 320/240
#
# AND DO NOT RAISE THIS TO 640x480 hoping to help the emitter model — it cannot use the
# pixels. The deployed FOMO model's input is 160x160 with resize mode "fit-shortest", so
# every frame we hand the brick is scaled to 160x160 before inference and 640x480 arrives
# identically to 320x240. To give the model more resolution, send it a 160x160 CROP of the
# lower frame (fit-shortest then does nothing, so the emitter lands 1:1 — 1.5x the model
# pixels) or retrain with a larger input in Edge Impulse. Neither is a capture-size change.
FRAME_W, FRAME_H = 320, 240


# Device names that are NOT cameras. The UNO Q's Venus hardware codec presents V4L2 nodes
# that look like capture devices to anything that only counts /dev/videoN.
_NOT_A_CAMERA = ("venus", "encoder", "decoder", "codec", "m2m", "jpeg")


def find_uvc_camera():
    """Return the /dev/video index of a real USB camera, or None.

    THE INDEX IS NOT STABLE. Verified 2026-08-18: on one boot the C310 was video2/video3 and
    the Venus codec was video0/video1; after unplugging the camera to mount it they SWAPPED.
    Opening a hardcoded index then fails, or worse succeeds on the codec and never delivers
    a frame.

    Identifies via SYSFS — /sys/class/video4linux/videoN/name — which matters for two
    reasons that the obvious approaches get wrong:

      * `v4l2-ctl` is NOT installed inside the app container (only on the host), so shelling
        out to it works when testing over ssh and silently fails where it counts.
      * Probing by opening each device FAILS WHILE THE CAMERA IS IN USE. The app's own camera
        loop holds it, so a probe-based search returns None exactly when the app most needs to
        re-find the camera. Reading a sysfs name needs no exclusive access.

    A UVC camera exposes two nodes (capture + metadata) with the same name; `index` tells them
    apart, so prefer index 0.
    """
    cands = []
    base = "/sys/class/video4linux"
    try:
        for node in sorted(os.listdir(base)):
            if not node.startswith("video"):
                continue
            try:
                with open(os.path.join(base, node, "name")) as f:
                    name = f.read().strip()
            except OSError:
                continue
            if any(bad in name.lower() for bad in _NOT_A_CAMERA):
                continue                      # a codec, not a camera
            try:
                with open(os.path.join(base, node, "index")) as f:
                    sub = int(f.read().strip())
            except (OSError, ValueError):
                sub = 0
            cands.append((sub, int(node[len("video"):]), name))
    except OSError:
        pass
    if cands:
        cands.sort()                          # index 0 first, then lowest device number
        return cands[0][1]

    # Last resort, for a bare host with an unusual sysfs: actually try to grab a frame.
    # Note this cannot succeed if something else already holds the camera.
    for idx in range(6):
        if not os.path.exists("/dev/video%d" % idx):
            continue
        cap = cv2.VideoCapture(idx)
        try:
            if cap.isOpened():
                ok, frame = cap.read()
                if ok and frame is not None:
                    return idx
        finally:
            cap.release()
    return None


class FrameSource:
    def __init__(self, source, loop=True, size=(FRAME_W, FRAME_H)):
        self.source = source
        self.loop = loop
        self.size = size
        self.resized = 0            # frames the driver gave us at the wrong size
        self._still = None
        self._cap = None

        is_image = isinstance(source, str) and source.lower().endswith(
            (".png", ".jpg", ".jpeg", ".bmp"))
        if is_image:
            self._still = cv2.imread(source)
            if self._still is None:
                raise FileNotFoundError("image not found: %s" % source)
        else:
            # int (webcam), stream URL, or video file all go through VideoCapture
            self._cap = cv2.VideoCapture(source)
            if not self._cap.isOpened():
                raise RuntimeError("could not open source: %r" % source)
            if self.size:
                # This WORKS on the C310 (verified with v4l2-ctl against the live device),
                # so we capture 320x240 natively: a quarter of the USB bandwidth of 640x480
                # and no per-frame resize. _fit() below stays as the belt-and-braces check
                # for a camera that accepts the request and ignores it.
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
            # NOTE: do NOT set CAP_PROP_BUFFERSIZE on this source. Tried it 2026-08-17
            # to stop partially-decoded frames; on the ESP32-CAM's MJPEG-over-HTTP the
            # FFmpeg backend delivered ONE frame after each open and then stalled —
            # the loop reopened every 5s, forever. Buffered-frame staleness is the
            # lesser problem; a stream that does not deliver is fatal.

    def _fit(self, frame):
        """Guarantee the pinned size, whatever the driver decided to give us."""
        if frame is None or not self.size:
            return frame
        if frame.shape[1] != self.size[0] or frame.shape[0] != self.size[1]:
            self.resized += 1
            frame = cv2.resize(frame, self.size, interpolation=cv2.INTER_AREA)
        return frame

    def read(self):
        """Return (ok, frame). The frame is always the pinned size."""
        if self._still is not None:
            return True, self._fit(self._still.copy())
        ok, frame = self._cap.read()
        if not ok and self.loop and isinstance(self.source, str) \
                and os.path.exists(self.source):
            # end of a video file -> rewind and retry (handy for looping demos)
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self._cap.read()
        return ok, self._fit(frame)

    def release(self):
        if self._cap is not None:
            self._cap.release()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.release()


class MjpegStream:
    """Read the camera's MJPEG stream ourselves and decode with a VALIDATING decoder.

    THE WHOLE POINT: cv2.VideoCapture uses FFmpeg, whose MJPEG decoder is permissive — it
    hands back a partially decoded frame rather than refusing it. That is the entire reason
    a corrupt frame ever reaches the robot as a green block. Measured on a real frame cut
    to 45% of its bytes:

        FFmpeg  (VideoCapture)  -> returns a frame, 58.3% green, silently accepted
        cv2.imdecode            -> returns None

    OpenCV's own JPEG decoder validates. So parse the multipart stream (trivial — the
    camera sends Content-Length per part) and decode each frame with imdecode: an
    incomplete JPEG is dropped instead of shown. Not a heuristic, not a green-pixel
    detector — a decoder refusing malformed data.

    This matters because the camera genuinely does emit incomplete JPEGs: with fb_count=2
    and CAMERA_GRAB_LATEST the driver can overwrite the buffer while app_httpd.cpp is
    still transmitting it (espressif/esp32-camera#417), and ~5% of frames lack the
    end-of-image marker (#252). Fixing that needs a cam reflash; this makes it harmless
    without one.

    `dropped` counts rejections, so the rate is visible rather than guessed at.
    """

    def __init__(self, url, timeout=5.0, size=(FRAME_W, FRAME_H)):
        self.url = url
        self.size = size
        self.dropped = 0
        self.decoded = 0
        self.resized = 0
        self._fp = urllib.request.urlopen(url, timeout=timeout)
        ctype = self._fp.headers.get("Content-Type", "")
        m = re.search(r'boundary=([^\s;]+)', ctype)
        if not m:
            self._fp.close()
            raise RuntimeError("not an MJPEG stream (Content-Type: %r)" % ctype)
        self._boundary = m.group(1).strip('"').encode()

    def read(self):
        """(ok, frame). An incomplete JPEG is dropped and reported as (False, None)."""
        try:
            raw = _next_jpeg(self._fp, self._boundary)
        except Exception:                       # noqa: BLE001 — treat as a dead stream
            return False, None
        if raw is None:
            return False, None
        frame = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.dropped += 1                   # incomplete/corrupt — never show it
            return False, None
        if self.size and (frame.shape[1] != self.size[0] or frame.shape[0] != self.size[1]):
            self.resized += 1
            frame = cv2.resize(frame, self.size, interpolation=cv2.INTER_AREA)
        self.decoded += 1
        return True, frame

    def release(self):
        try:
            self._fp.close()
        except Exception:                       # noqa: BLE001
            pass


def _next_jpeg(fp, boundary):
    """Pull one part's JPEG bytes from a multipart stream. None at end of stream.

    Split out so it can be tested against a synthetic stream without a camera.
    """
    # walk to the next boundary
    while True:
        line = fp.readline()
        if not line:
            return None
        if boundary in line:
            break
    length = None
    while True:                                 # part headers, blank line ends them
        line = fp.readline()
        if not line:
            return None
        if line in (b"\r\n", b"\n"):
            break
        if line.lower().startswith(b"content-length:"):
            try:
                length = int(line.split(b":", 1)[1].strip())
            except ValueError:
                length = None
    if length is None:
        return None                             # no length: cannot know where it ends
    buf = fp.read(length)
    return buf if buf and len(buf) == length else None


class FrameBus:
    """One thread reads the camera; everyone else takes the LATEST frame, with its age.

    WHY A THREAD. The control loop blocks for seconds at a time — 2s of plant dwell at each
    emitter, ~1.5s per gyro pivot, ~1.7s creeping onto a lateral. While it blocks, nobody
    reads the socket, the TCP window fills, and an MJPEG camera that cannot complete a send
    abandons the frame: 'Stream ends prematurely at 5675' in the log, then a reopen. Reading
    in a thread means the transport keeps flowing no matter what the robot is doing.

    WHY TIMESTAMPS. A frame with no age is the more dangerous half of the problem. After a
    2s stop, the next read returns whatever was buffered — a view from BEFORE the stop —
    and nothing about it looks wrong, so the robot would steer on where the tube used to
    be. Publishing (captured_at, frame) lets a caller refuse it, which is a thing it simply
    could not do before.

    This bus intentionally KEEPS ONLY THE NEWEST FRAME. Queueing would preserve exactly the
    staleness we are trying to eliminate.

    Note this replaces the old flush-by-timing hack, which read frames until one took
    >40ms and called that the live edge. It was guessing at the buffer depth; a capture
    time is the real answer.
    """

    def __init__(self, source, reopen_after_s=5.0, on_log=None):
        self.source = source
        self.reopen_after_s = reopen_after_s
        self._log = on_log or (lambda m: None)
        self._src = None
        self._lock = threading.Lock()
        self._t = 0.0                 # capture time of the newest frame
        self._frame = None
        self._stop = threading.Event()
        self._thread = None
        self.frames = 0               # delivered by the camera
        self.reopens = 0
        self.open_failures = 0
        self._interval = None         # EMA of the gap between frames, seconds

    @property
    def resized(self):
        """Frames the driver delivered at the wrong size and we had to scale."""
        return getattr(self._src, "resized", 0)

    @property
    def dropped(self):
        """Frames the decoder refused as incomplete — the corruption rate, measured."""
        return getattr(self._src, "dropped", 0)

    # ── lifecycle ──────────────────────────────────────────────────────────
    @staticmethod
    def _open(source):
        """An MJPEG URL gets the VALIDATING reader; files and webcams keep FrameSource.

        A USB camera may arrive as an int, a bare digit string from the UI, or a
        /dev/videoN path — OpenCV wants the INDEX for a V4L2 device, so normalise here
        rather than making every caller know that.
        """
        if isinstance(source, str):
            if source.startswith("http"):
                return MjpegStream(source)
            if source.lower() in ("usb", "auto", "webcam"):
                idx = find_uvc_camera()
                if idx is None:
                    raise RuntimeError("no USB camera found (checked /dev/video0-5)")
                return FrameSource(idx)
            if source.startswith("/dev/video"):
                return FrameSource(int(source[len("/dev/video"):]))
            if source.isdigit():
                return FrameSource(int(source))
        return FrameSource(source)

    def start(self):
        self._src = self._open(self.source)       # raises if it cannot open at all
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._src is not None:
            try:
                self._src.release()
            except Exception:                     # noqa: BLE001
                pass

    def alive(self):
        return self._thread is not None and self._thread.is_alive()

    # ── the reader ─────────────────────────────────────────────────────────
    def _run(self):
        last_ok = time.time()
        while not self._stop.is_set():
            ok, frame = (False, None)
            try:
                ok, frame = self._src.read()
            except Exception as e:                # noqa: BLE001
                self._log("camera read raised: %s" % e)
            if ok and frame is not None:
                now = time.time()
                with self._lock:
                    if self._t:       # learn how fast this camera actually delivers
                        gap = now - self._t
                        if 0 < gap < 5:
                            self._interval = gap if self._interval is None \
                                else 0.8 * self._interval + 0.2 * gap
                    self._t, self._frame = now, frame
                self.frames += 1
                last_ok = time.time()
                continue
            # No frame. Give the stream a moment, then rebuild it — this is the only
            # place the capture object is touched, so no other code can race it.
            if time.time() - last_ok < self.reopen_after_s:
                time.sleep(0.02)
                continue
            self.reopens += 1
            self._log("camera stream stale for %.0fs — reopening (#%d)"
                      % (self.reopen_after_s, self.reopens))
            try:
                self._src.release()
            except Exception:                     # noqa: BLE001
                pass
            try:
                self._src = self._open(self.source)
                # DISCARD THE FIRST FRAME after a reopen, always. It is not a heuristic
                # judgement about corruption: a partially-received JPEG decoded into a
                # freshly zero-filled buffer is what produces the green block (zeros in
                # YUV are RGB(0,135,0)), and the first frame after an open is exactly when
                # the buffer is fresh. One frame is nothing at 15-25 fps.
                self._src.read()
                self._log("camera stream reopened")
            except Exception as e:                # noqa: BLE001
                self.open_failures += 1
                self._log("camera reopen failed: %s" % e)
                time.sleep(0.5)
            last_ok = time.time()

    # ── consumers ──────────────────────────────────────────────────────────
    def latest(self):
        """(captured_at, frame) — may be OLD. Check the age before acting on it."""
        with self._lock:
            return self._t, (None if self._frame is None else self._frame)

    def age(self):
        with self._lock:
            return float("inf") if self._t == 0.0 else time.time() - self._t

    def interval(self):
        """Measured seconds between frames, or None before two have arrived."""
        with self._lock:
            return self._interval

    def stale_after(self, floor=0.15, mult=6.0, cap=3.0):
        """How old a frame must be before it means the stream has DIED.

        MEASURED, not assumed. A fixed 0.25s limit looked sensible for a 10fps camera and
        was catastrophic for this one: it delivers ~1.4fps, so a new frame arrives every
        ~700ms and EVERY frame was already 'too old' — the control loop stopped the motors
        between frames and the robot spluttered instead of driving. The gate exists to catch
        a dead stream, not to punish a slow one, so it scales with what the camera is
        actually doing and only the floor and cap are fixed.

        RETUNED 2026-08-18, from floor=0.35/mult=3 to floor=0.15/mult=6. The old FLOOR was
        the binding constraint on the USB camera and nobody had noticed: 3 x 33.5ms is 0.10s,
        so the floor decided everything, and 0.35s at 0.170 m/s is SIX CENTIMETRES of blind
        driving before the motors stop. That floor was sized to protect the 1.4fps camera —
        but `mult` already protects a slow camera far better than a fixed floor can (6 x
        710ms saturates the 3s cap), so the floor was only ever binding on a FAST camera,
        where it is least justified.

        Now: 0.20s at 30fps, which is six frames and 3.4cm. Six frames of tolerance is
        deliberately generous against jitter — the measured worst single read is 37ms — and
        it is the same order as the tube-grace window, so the two fail-safes agree about how
        long the robot may travel on nothing. The floor of 0.15 exists only for a camera
        faster than 40fps, where 6 x interval would be tighter than any real jitter margin.

        DO NOT tighten this to 3 x interval to save the last centimetre. That is the shape
        of the mistake that caused the spluttering: the gate must catch a DEAD stream, and a
        stream that hiccups for three frames is not dead.
        """
        iv = self.interval()
        if iv is None:
            return cap
        return max(floor, min(cap, mult * iv))

    def wait_fresh(self, after_t=None, timeout=1.5):
        """Block until a frame CAPTURED AFTER `after_t` exists. (t, frame) or (0, None).

        This is what to call after anything that blocked — a plant dwell, a pivot, a creep.
        It guarantees the frame shows the world as it is now, rather than as it was when the
        robot stopped moving.
        """
        after_t = time.time() if after_t is None else after_t
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._t > after_t and self._frame is not None:
                    return self._t, self._frame
            time.sleep(0.01)
        return 0.0, None
