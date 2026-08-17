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
import cv2


class FrameSource:
    def __init__(self, source, loop=True):
        self.source = source
        self.loop = loop
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
            # NOTE: do NOT set CAP_PROP_BUFFERSIZE on this source. Tried it 2026-08-17
            # to stop partially-decoded frames; on the ESP32-CAM's MJPEG-over-HTTP the
            # FFmpeg backend delivered ONE frame after each open and then stalled —
            # the loop reopened every 5s, forever. Buffered-frame staleness is the
            # lesser problem; a stream that does not deliver is fatal.

    def read(self):
        """Return (ok, frame)."""
        if self._still is not None:
            return True, self._still.copy()
        ok, frame = self._cap.read()
        if not ok and self.loop and isinstance(self.source, str) \
                and os.path.exists(self.source):
            # end of a video file -> rewind and retry (handy for looping demos)
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self._cap.read()
        return ok, frame

    def release(self):
        if self._cap is not None:
            self._cap.release()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.release()
