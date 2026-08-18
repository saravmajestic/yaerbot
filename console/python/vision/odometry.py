"""Visual odometry — measure the ground the robot actually covered, instead of assuming it.

WHY. `travelled` is integrated as wall-clock x _DRIP_SPEED_MPS, a single hand-tuned constant.
That gates end-of-row and the replant floor, and it is the root cause of the run that was told
to cover 5m and covered about 7m. The error cannot be tuned away because it depends on the
soil: speed near the PWM deadband is dominated by stiction, so the same duty travels different
distances on different ground, and two observation-based calibrations of the same constant
disagreed by 40% (0.161 vs 0.225 m/s) purely because the robot stuttered more in one run.

The USB camera makes measuring it practical for the first time. Consecutive frames overlap
~97% at 26-30fps, which is close to ideal for phase correlation: the shift is large enough to
measure and small enough to stay unambiguous.

MEASURED on 2085 consecutive real frames (2026-08-18): response median 0.641, |dy| median
9.05 px/frame, and frames where the robot was stationary read 0.06px — so it separates moving
from stopped cleanly, which is the property that matters for integrating distance.

NOT YET CALIBRATED AGAINST CREEP. That pass was driven manually, i.e. at the manual drive duty
rather than PWM 55, so it says nothing about whether 0.170 m/s is right. Until a creep-speed run
exists this class is MEASURE-ONLY: the camera loop logs its distance beside the time-integrated
one and acts on neither. Comparing the two over a known distance is what promotes it.
"""

import cv2
import numpy as np

# Depth scale: how many pixels of vertical image shift one metre of forward travel produces.
# MEASURED, not assumed — the 16mm drip tube is a ruler of known width in every frame, and it
# reads 20.9px across at the frame bottom (0.76 mm/px) and 13.0px at the top (1.23 mm/px). A
# 22cm visible strip over 240 rows implies 1.19 mm/px mean, which agrees. See main.py's
# _PX_PER_M_DEPTH, which must stay in step with this.
PX_PER_M_DEPTH = 1090.0

# Below this correlation response the match is not trustworthy — featureless ground, motion
# blur, or a frame the camera mangled. Measured p10 on real frames is 0.269, so 0.15 rejects
# the genuinely bad without discarding ordinary soil.
MIN_RESPONSE = 0.15

# A single frame pair cannot imply more travel than the robot can physically manage. At 0.30 m/s
# and a 40ms frame that is 12mm = 13px; 60px is a generous ceiling that still rejects a
# correlation that locked onto the wrong feature entirely.
MAX_SHIFT_PX = 60.0

# NOISE FLOOR, and the reason this class was over-reading by 66%.
#
# The first version integrated abs(dy). That is a RECTIFIED integral: vibration, rotation and
# correlation noise all ADD and never subtract, so on soil the total climbs monotonically whether
# or not the robot is moving. Measured against a tape on 2026-08-18 it read 0.274 m/s where the
# truth was 0.165 — 66% high — and it read a consistent 0.62 ratio across every log window, which
# looked like a real measurement and was error accumulation.
#
# Two changes fix it. The travel is now integrated SIGNED, so noise cancels instead of piling up
# and the distance is the magnitude of the NET displacement. And a shift smaller than this floor
# contributes nothing at all: real travel at 0.165 m/s over a 34ms frame is
# 0.165 * 0.034 * 1090 = 6.1px, so 1.5px is comfortably below the signal and above the jitter.
#
# The old unit test could not catch this: it fed a perfectly static DUPLICATE frame, which has no
# noise to rectify. A stationary robot on real soil vibrates, and that is the case that mattered.
MIN_SHIFT_PX = 1.5


class FlowOdometer:
    """Integrate forward travel from consecutive frames.

        odo = FlowOdometer()
        odo.update(frame)        # call once per NEW frame
        odo.distance_m           # metres since the last reset

    Uses the LOWER HALF of the frame: it is nearest the camera, so its scale is best known and
    least distorted by perspective, and it is where soil texture is sharpest.
    """

    def __init__(self, px_per_m=PX_PER_M_DEPTH, roi=(0.5, 1.0)):
        self.px_per_m = float(px_per_m)
        self._roi = roi
        self._prev = None
        self._win = None
        self._net_dy = 0.0          # SIGNED accumulator; noise cancels here
        self.distance_m = 0.0
        self.updates = 0
        self.rejected = 0        # pairs whose correlation was not trustworthy
        self.last_shift_px = 0.0
        self.last_response = 0.0

    def _prep(self, frame):
        h = frame.shape[0]
        lo, hi = int(h * self._roi[0]), int(h * self._roi[1])
        g = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return g[lo:hi, :].astype(np.float32)

    def reset(self):
        self.distance_m = 0.0
        self._net_dy = 0.0
        self._prev = None
        self.updates = 0
        self.rejected = 0

    def update(self, frame):
        """Add this frame's travel. Returns the metres added (0.0 if not usable)."""
        cur = self._prep(frame)
        if self._prev is None or self._prev.shape != cur.shape:
            self._prev = cur
            self._win = cv2.createHanningWindow((cur.shape[1], cur.shape[0]), cv2.CV_32F)
            return 0.0
        try:
            (_dx, dy), response = cv2.phaseCorrelate(self._prev, cur, self._win)
        except Exception:                              # noqa: BLE001 — never break the loop
            self._prev = cur
            self.rejected += 1
            return 0.0
        self._prev = cur
        self.last_shift_px, self.last_response = abs(dy), response
        # REJECT RATHER THAN GUESS. A bad correlation that is integrated anyway accumulates
        # silently into the distance the run is gated on, which is the exact failure mode this
        # class exists to remove.
        if response < MIN_RESPONSE or abs(dy) > MAX_SHIFT_PX:
            self.rejected += 1
            return 0.0
        if abs(dy) < MIN_SHIFT_PX:
            # Below the noise floor: contribute NOTHING rather than a small positive amount.
            # Summing these is what made a parked robot accumulate phantom travel.
            self.updates += 1
            return 0.0
        before = abs(self._net_dy)
        self._net_dy += dy                      # signed, so jitter cancels over time
        self.distance_m = abs(self._net_dy) / self.px_per_m
        self.updates += 1
        return (abs(self._net_dy) - before) / self.px_per_m

    def stats(self):
        return {"distance_m": round(self.distance_m, 3), "updates": self.updates,
                "rejected": self.rejected,
                "last_shift_px": round(self.last_shift_px, 2),
                "last_response": round(self.last_response, 3)}
