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
                  pair_win=45):
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

    peak = int(np.argmax(mag))
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


def detect_tube(frame, bands=4, max_drift_px=45, **kw):
    """Find the drip line being followed (near-vertical) and return a steering correction.

    Returns dict:
      found       — was a tube found
      correction  — -5..+5 steering hint (+ = tube is RIGHT of center -> steer right)
      offset_px   — tube_x - frame_center_x (+ = right)
      tube_x      — estimated tube column (px)
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
    # ORGANISED ALONG, NOT ACROSS. Without this, a frame containing no vertical
    # structure at all still yields a peak — the strongest bit of noise — and the
    # follow loop steers at it. Symmetric with detect_crossing, and it means "found"
    # is a claim about the frame, not just about the profile's maximum.
    across = _profile_line(gray, 1, **kw)
    if across is not None and across["strength"] > whole["strength"]:
        return none

    # VERTICAL COHERENCE — the test that separates the tube from a shadow.
    # The row we follow runs the full height of the frame. A clod's shadow edge, or the
    # boundary of a sunlit patch, does not: it occupies part of the frame and wanders.
    # A strength threshold cannot tell them apart (measured: real tubes 3.0-4.8 sigma,
    # shadow edges 2.4-3.4 — overlapping), and neither can width. But splitting the
    # frame into quarters and asking each to find the SAME line does, cleanly: across
    # every real frame we have, all four quarters agree to within 34px, while shadow
    # edges scatter — 19/173/99/181 on the frame that sent an alignment loop chasing
    # bare soil. So coherence is required for `found`, not merely used for the angle.
    xs, ys = [], []
    for i in range(bands):
        seg = gray[int(h * i / bands):int(h * (i + 1) / bands), :]
        r = _profile_line(seg, 0, **kw)
        if r is None:
            return none                       # a quarter with no line at all: not a row
        xs.append(r["pos"])
        ys.append(h * (i + 0.5) / bands)
    if max(xs) - min(xs) > max_drift_px:
        return none                           # they disagree: local feature, not a row

    angle = round(float(np.degrees(np.arctan2(xs[-1] - xs[0], ys[-1] - ys[0]))), 1)

    offset = whole["pos"] - w / 2.0
    return {"found": True,
            "correction": round(float(np.clip(offset / (w / 2.0) * 5.0, -5, 5)), 2),
            "offset_px": round(offset, 1), "tube_x": round(whole["pos"], 1),
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
