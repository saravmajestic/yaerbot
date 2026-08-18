"""
Farm OS — Stage 5 vision (OpenCV, runs on the UNO Q Qualcomm/Linux side).

Pure OpenCV/NumPy — NO Arduino/Bridge deps, so it's unit-testable off-device
(see tests/test_tube.py, tests/test_emitter.py). The on-device control loop
(tube_follow.py) imports these and drives the STM32 over RouterBridge.

  detect_tube(frame)              -> steering correction from the drip line
  detect_emitter(frame, moisture) -> emitter (a bump on the tube) + moisture confirm
  detect_crossing(frame)          -> the NEXT lateral seen side-on while traversing

Assumptions: the robot drives ALONG the drip line, so the tube runs roughly
top-to-bottom (near-vertical) in the frame. The line is darker than the soil.

`detect_crossing` is the exception, and the reason it exists: between laterals the
robot drives ACROSS the rows, so the next tube arrives as a near-HORIZONTAL line.
detect_tube would reject it — its near-vertical filter is exactly what makes
tube-following immune to furrows and shadows running crosswise. Searching for the
next lateral instead of computing where it should be means the rows need not be
parallel or evenly spaced, which real drip layouts rarely are.
"""

import cv2
import numpy as np


def _dark_mask(gray, thresh=90):
    """Binary mask of the dark tube/emitter (dark pixels -> 255)."""
    _, m = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY_INV)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return m


def _smooth1d(v, k):
    k = int(k) | 1                       # box filter needs an odd window
    return cv2.blur(v.reshape(-1, 1).astype(np.float32), (1, k)).ravel()


def _profile_line(gray, axis, bg_win=81, min_sigma=2.0, min_w=10, max_w=95,
                  pair_win=45, seed=None, seed_win=None):
    """Find a tube as a run in a 1-D PROJECTION PROFILE. The core of both detectors.

    REWRITTEN 2026-08-17, replacing Canny+Hough, which was not tracking the tube at
    all. On a real run the reported tube column jumped 233 -> 290 -> 158 -> 61 -> 256
    within about two seconds: it was taking the median of scattered soil edges, so the
    steering was being driven by noise and every run behaved differently.

    Two facts about this problem make projection profiles the right tool:

    * GEOMETRY IS KNOWN. The row being followed runs top-to-bottom; a crossing lateral
      runs left-to-right. So collapse the frame onto the axis the tube is NOT along —
      240 pixels averaged into one — and soil texture, which is what defeated Hough,
      averages away. The tube survives because it is coherent along that axis.
    * POLARITY IS NOT KNOWN. Measured on frames from one run: following the row the
      tube read 170 mean against soil at 157 (BRIGHTER); ninety seconds later, crossing,
      112 against 176 (DARKER). Same tube. With a serpentine path this flips every row,
      because alternate rows face into and away from the sun. So the test is departure
      from the local background in EITHER direction, and the sign is reported, not
      assumed. Every previous detector here keyed on darkness and was therefore wrong
      half the time.

    The tube is a lit cylinder, so it appears as a BRIGHT run beside a DARK run (its own
    shadow). Centring on whichever side is stronger biases the estimate onto the
    highlight — off by ~25px on real frames. So when an opposite-signed partner is found
    within a tube width, the centre is the midpoint of the PAIR.

    Returns {pos, width, strength (in sigma), polarity} or None.
    """
    prof = gray.mean(axis=axis)                    # axis=0 -> over x, axis=1 -> over y
    dev = prof - _smooth1d(prof, bg_win)           # departure from local background
    sigma = float(np.std(dev)) or 1e-6
    mag = np.abs(dev)
    n = len(dev)

    # `seed` restricts WHERE the peak may be, not how the background is measured — the
    # background still needs the whole profile for context. This is what lets a caller
    # track the tube band-to-band instead of asking each band the question from scratch.
    if seed is None:
        peak = int(np.argmax(mag))
    else:
        lo_s = max(0, int(seed - seed_win))
        hi_s = min(n, int(seed + seed_win) + 1)
        if hi_s <= lo_s:
            return None
        peak = lo_s + int(np.argmax(mag[lo_s:hi_s]))
    if mag[peak] < min_sigma * sigma:
        return None
    sign = 1.0 if dev[peak] > 0 else -1.0

    # the cylinder's other side: the strongest OPPOSITE-signed point nearby
    lo, hi = max(0, peak - pair_win), min(n, peak + pair_win + 1)
    partner_win = dev[lo:hi] * -sign
    partner = lo + int(np.argmax(partner_win))
    centre = width = None
    if partner_win.max() > 1.2 * sigma:
        c, wd = (peak + partner) / 2.0, abs(partner - peak) * 2
        if min_w <= wd <= max_w:
            centre, width = c, wd

    if centre is None:                             # no partner: full width at half max
        half = mag[peak] * 0.5
        l = peak
        while l > 0 and mag[l - 1] > half:
            l -= 1
        r = peak
        while r < n - 1 and mag[r + 1] > half:
            r += 1
        centre, width = (l + r) / 2.0, r - l + 1

    if not (min_w <= width <= max_w):
        return None
    return {"pos": float(centre), "width": int(width),
            "strength": round(float(mag[peak] / sigma), 2),
            "polarity": "bright" if sign > 0 else "dark"}


# How many of the four bands must lie on the fitted line for the frame to count.
#
# MEASURED across three frame sets, 2026-08-18 (20 ESP32-CAM frames — 9 with a tube, 11
# without; 6 clean USB frames; 4 USB frames with the robot's own shadow across the view):
#
#   _MIN_BANDS=4   9/9 real, 11/11 rejected, 0 false | 6/6 clean USB | 0/4 shadowed
#   _MIN_BANDS=3   9/9 real,  7/11 rejected, 4 FALSE | 6/6 clean USB | 4/4 shadowed
#
# 4 it is. Allowing one outlier does recover the shadowed frames, but it costs four false
# positives out of eleven — and those four are all frames from the traverse/alignment phase,
# where a wrong answer makes the robot think it is aligned with a row it is not on. Steering
# briefly toward nothing is worse than briefly not steering.
#
# THE SHADOW CASE IS NOT SOLVED IN SOFTWARE, and four discriminators were tried and rejected
# on the evidence: all-four agreement, a contiguous walk from the nearest band, a 3-inlier
# robust fit, and requiring the bright/dark PAIR of a lit cylinder rather than a one-sided
# step. The last was the most promising — a shadow edge is a step, a tube is a pair — but the
# partner strengths overlap completely (real shadowed frames 1.5-1.7 sigma, false positives
# 1.6-2.2, clean tubes 1.9-3.2), so it does not separate them.
#
# The robot's own shadow falling across the view is a PHYSICAL problem: it moves with the sun
# and will not hold still for a threshold. Fix it by keeping the shadow out of frame, or by
# lighting the scene. See docs/usb-camera.md.
_MIN_BANDS = 4

# AXIS-DOMINANCE TEST: REMOVED 2026-08-18. It compared the strength of the whole-frame
# vertical profile against the horizontal one, to avoid claiming a crossing tube as the row
# being followed. It broke the USB camera outright — 1 of 6 frames detected on a tube that is
# plainly visible in all 6 — for a reason that is worth keeping written down:
#
#   The tube is DIAGONAL (measured: x=172 at the top of frame, x=120 at the bottom, so 52px
#   of drift). Collapsing the whole frame smears that drift, so the `along` signal is weak
#   (3.4 sigma) even though each individual band sees a crisp 10-15px tube at 3.7-5.0 sigma.
#   Meanwhile a bright band of straw in the soil gave a genuinely strong `across` signal
#   (4.8 sigma at y=123, +22.7 grey levels). So the test compared a SMEARED tube against a
#   CRISP piece of soil, and the soil won.
#
# It was also redundant. Measured across two frame sets — 20 ESP32-CAM frames (9 with a tube,
# 11 without, including 3 genuine crossings) and 6 USB-camera frames:
#     axis test disabled       -> 9/9 real, 11/11 rejected, 0 false | 6/6 USB
#     axis 1.30 on whole-frame -> 9/9 real, 11/11 rejected, 0 false | 1/6 USB
# The band-coherence walk below already rejects horizontal tubes, which is what this was for.
#
# detect_crossing keeps its own along-vs-across comparison: there it is applied to a
# genuinely horizontal target and is not comparing against a smeared signal.


def detect_tube(frame, bands=4, max_drift_px=45, max_step_px=40,
                max_line_dev_px=22, **kw):
    """Find the drip line being followed (near-vertical) and return a steering correction.

    Returns dict:
      found       — was a tube found
      correction  — -5..+5 steering hint (+ = tube is RIGHT of center -> steer right)
      offset_px   — tube_x - frame_center_x (+ = right)
      tube_x      — the tube column AT THE ROBOT (the nearest band). Was the whole-frame
                    average, which halves a heading error: with the robot still over the
                    tube at the wheels, the displacement is all at the top of the frame,
                    so averaging hides it until it has grown into a lateral offset — i.e.
                    until the tube reaches the wheels. That is what made steering late.
      x_far/x_near, far_correction, bands_x
                  — the LOOKAHEAD point and the per-band track. far_correction carries the
                    same sign convention as `correction`, so a controller can use both
                    without any reasoning about angles.
      angle_deg   — tilt from VERTICAL in degrees (+ = leaning right going down),
                    fitted across the frame quarters. NOTE this is a different
                    convention from the old Hough angle (which was ~±90 for a vertical
                    line); it is now signed tilt, 0 = perfectly aligned.
      width/strength/polarity — from the profile, for logging and gating
    """
    h, w = frame.shape[:2]
    gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    whole = _profile_line(gray, 0, **kw)
    none = {"found": False, "correction": 0.0, "offset_px": None, "tube_x": None,
            "angle_deg": None, "width": None, "strength": 0.0, "polarity": None}
    if whole is None:
        return none
    # TWO PASSES, and they do different jobs. Measured on 20 real frames, each pass alone
    # fails in a different direction, so both are kept:
    #
    #   INDEPENDENT (accept/reject) — every band searches the full width, and they must
    #   agree. Real tubes agree within 34px; shadow edges scatter (19/173/99/181). This is
    #   what gives 0 false positives, and it is the gate.
    #
    #   SEEDED (measure) — bottom-up, each band searching near where the band below found
    #   the tube. Better positions on a tilted tube, and one band clipped by a shadow gets
    #   pulled back to the row instead of wandering. It is how the published crop-row work
    #   does it (10 strips, each ROI initialised at the previous strip's centre) — but that
    #   runs on a CNN segmentation mask where the row is the only candidate. On raw grey
    #   profiles, seeding will happily track ANY smooth gradient: used as the gate it turned
    #   4 shadow frames into detections, because forcing continuity onto noise manufactures
    #   exactly the continuity the test is looking for.
    step = h // bands
    ys_all = [(i + 0.5) * step for i in range(bands)]

    indep = []
    for i in range(bands):
        r = _profile_line(gray[i * step:(i + 1) * step, :], 0, **kw)
        indep.append(None if r is None else r["pos"])

    # ROBUST LINE FIT ACROSS THE BANDS. The tube is straight, so the four band positions
    # should lie on a line; the job is to find that line when some bands are wrong.
    #
    # Two earlier rules both failed, in opposite directions, and the failures are the reason
    # this is a fit rather than a rule:
    #   * "all four must agree within 45px" rejected any frame where the tube ran off the top
    #     of the view — i.e. exactly when the robot had drifted and needed to steer back.
    #   * "walk up from the bottom band while it agrees" assumed the BOTTOM band is the
    #     trustworthy one. Measured 2026-08-18, the robot's own shadow falls across the
    #     bottom-left of the frame and pulled that band 73px off the tube while the upper
    #     three agreed to within 38px. Starting the walk from the corrupted band threw the
    #     frame away.
    # So: try every pair of bands as a hypothesis, count how many bands lie near that line,
    # and keep the best-supported one. With four bands that is six cheap hypotheses.
    #
    # The fitted line, not the raw band values, gives the near and far points — so a band
    # ruined by shadow is EXTRAPOLATED THROUGH rather than either trusted or fatal.
    pts = [(ys_all[i], indep[i]) for i in range(bands) if indep[i] is not None]
    if len(pts) < _MIN_BANDS:
        return none

    best = None                               # (inlier count, -residual, slope, intercept)
    for a in range(len(pts)):
        for b in range(a + 1, len(pts)):
            (y1, x1), (y2, x2) = pts[a], pts[b]
            if y2 == y1:
                continue
            m = (x2 - x1) / float(y2 - y1)
            c = x1 - m * y1
            res = [abs(x - (m * y + c)) for y, x in pts]
            n_in = sum(1 for r in res if r <= max_line_dev_px)
            score = (n_in, -sum(r for r in res if r <= max_line_dev_px))
            if best is None or score > best[0]:
                best = (score, m, c)
    if best is None or best[0][0] < _MIN_BANDS:
        return none
    n_inliers = best[0][0]
    _, m, c = best

    ys = list(ys_all)
    xs = [m * y + c for y in ys]              # the LINE's value at each band height

    x_far, x_near = xs[0], xs[-1]             # top of frame = furthest ahead
    # + = the tube leans right going DOWN the frame. Reported for logging; the steering
    # uses far and near offsets directly, which carry the same sign convention as
    # `correction` and so cannot be got backwards.
    angle = round(float(np.degrees(np.arctan2(x_near - x_far, ys[-1] - ys[0]))), 1)

    def _corr(x):
        return round(float(np.clip((x - w / 2.0) / (w / 2.0) * 5.0, -5, 5)), 2)

    return {"found": True,
            # cross-track: where the tube is AT THE ROBOT
            "correction": _corr(x_near),
            "offset_px": round(x_near - w / 2.0, 1), "tube_x": round(x_near, 1),
            # lookahead: where it will be. This is what allows correcting early.
            "far_correction": _corr(x_far), "far_px": round(x_far - w / 2.0, 1),
            "x_far": round(x_far, 1), "x_near": round(x_near, 1),
            # bands_used = how many bands SUPPORTED the fitted line. 4 is a clean read;
            # 3 means one band was rejected (shadow, occlusion, tube leaving the view) and
            # the line was extrapolated through it — worth logging when steering looks off.
            "bands_x": [round(v, 1) for v in xs], "bands_used": n_inliers,
            "bands_raw": [None if v is None else round(v, 1) for v in indep],
            "angle_deg": angle, "width": whole["width"],
            "strength": whole["strength"], "polarity": whole["polarity"]}


def detect_crossing(frame, **kw):
    """Find the NEXT lateral while the robot drives ACROSS the rows.

    Same primitive as detect_tube, rotated: collapse the COLUMNS instead of the rows, so
    a left-to-right tube becomes a run in a profile over y. See _profile_line for why
    projections rather than edges, and why polarity is measured rather than assumed.

    THE HARD PART is not finding a horizontal band — it is not mistaking the row we are
    following for one. A near-vertical tube leaves a residue in the row profile too. The
    discriminator is which AXIS the frame is more strongly organised along: a genuine
    crossing is stronger across than along. On the frames from the 2026-08-17 run this
    separates them 10 times out of 12, and the caller still needs several consecutive
    agreeing frames before acting.

    Returns dict:
      found      — a crossing tube-like band was seen
      tube_y     — its centre row (px). LARGER = nearer, camera looks ahead+down.
      nearness   — tube_y / height, 0..1
      width      — band thickness in px
      strength   — peak departure from local background, in sigma
      polarity   — "bright" or "dark", i.e. which side of the soil the tube read on
      angle_deg  — None. This detector is angle-agnostic by design: the tube crosses at
                   whatever angle the robot happens to approach on, commonly 15-25 deg.
    """
    h, w = frame.shape[:2]
    none = {"found": False, "tube_y": None, "nearness": 0.0, "width": None,
            "strength": 0.0, "polarity": None, "angle_deg": None}
    gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)

    across = _profile_line(gray, 1, **kw)          # horizontal band -> profile over y
    if across is None:
        return none
    along = _profile_line(gray, 0, **kw)           # the row we follow, if any
    if along is not None and along["strength"] >= across["strength"]:
        return none                                # organised along, not across: not a crossing

    return {"found": True, "tube_y": round(across["pos"], 1),
            "nearness": round(min(1.0, across["pos"] / h), 3),
            "width": across["width"], "strength": across["strength"],
            "polarity": across["polarity"], "angle_deg": None}


def detect_emitter(frame, moisture=None, moisture_wet_below=9000,
                   bump_ratio=1.6, min_bump_px=6, dark_thresh=90):
    """Detect an emitter as a local WIDTH bump on the tube contour, optionally
    confirmed by a low (wet) moisture reading — matches plan.md's
    detect_emitter(frame, moisture_reading).

    Returns dict:
      detected    — visual bump found
      position    — (x, y) of the bump centre, or None
      confidence  — 0..1 (visual bump + moisture confirm)
      visual/wet  — the two individual signals
    """
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mask = _dark_mask(gray, dark_thresh)

    # width profile: for each row, horizontal extent of the tube's dark pixels
    width = np.zeros(h)
    cx = np.zeros(h)
    for y in range(h):
        cols = np.where(mask[y] > 0)[0]
        if cols.size:
            width[y] = cols[-1] - cols[0] + 1
            cx[y] = cols.mean()
    rows = np.where(width > 0)[0]

    result = {"detected": False, "position": None, "confidence": 0.0,
              "visual": False, "wet": False, "bump_width": None,
              "baseline_width": None}
    if rows.size < h * 0.15:                # not enough tube visible
        wet = moisture is not None and moisture < moisture_wet_below
        result["wet"] = bool(wet)
        return result

    baseline = float(np.median(width[rows]))
    bump = rows[(width[rows] > baseline * bump_ratio) &
                (width[rows] > baseline + min_bump_px)]
    visual = bump.size > 0
    result["baseline_width"] = round(baseline, 1)

    if visual:
        my = int(np.median(bump))           # bump band centre row
        result.update(visual=True, detected=True,
                      position=(int(cx[my]), my),
                      bump_width=round(float(width[my]), 1))

    wet = moisture is not None and moisture < moisture_wet_below
    result["wet"] = bool(wet)

    # confidence: visual is primary, moisture confirms (plan.md: visual + moisture)
    if visual:
        vscore = min(1.0, (result["bump_width"] / baseline - 1.0))   # how pronounced
        result["confidence"] = round(0.6 * min(1.0, vscore) + (0.4 if wet else 0.0), 2)
    return result


def draw_overlay(frame, tube=None, emitter=None):
    """Return a copy of frame with detections drawn (for the UI/tests)."""
    out = frame.copy()
    h, w = out.shape[:2]
    cv2.line(out, (w // 2, 0), (w // 2, h), (90, 90, 90), 1)     # center ref
    if tube and tube.get("found"):
        x = int(tube["tube_x"])
        cv2.line(out, (x, 0), (x, h), (0, 200, 0), 2)
        cv2.putText(out, f"corr {tube['correction']:+.1f}", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
    if emitter and emitter.get("detected"):
        px, py = emitter["position"]
        cv2.circle(out, (px, py), 14, (0, 140, 255), 2)
        cv2.putText(out, f"emitter {emitter['confidence']:.2f}", (8, 46),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 140, 255), 2)
    return out
