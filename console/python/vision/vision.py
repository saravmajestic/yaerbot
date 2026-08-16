"""
Farm OS — Stage 5 vision (OpenCV, runs on the UNO Q Qualcomm/Linux side).

Pure OpenCV/NumPy — NO Arduino/Bridge deps, so it's unit-testable off-device
(see tests/test_tube.py, tests/test_emitter.py). The on-device control loop
(tube_follow.py) imports these and drives the STM32 over RouterBridge.

  detect_tube(frame)              -> steering correction from the drip line
  detect_emitter(frame, moisture) -> emitter (a bump on the tube) + moisture confirm

Assumptions: the robot drives ALONG the drip line, so the tube runs roughly
top-to-bottom (near-vertical) in the frame. The line is darker than the soil.
"""

import cv2
import numpy as np


def _dark_mask(gray, thresh=90):
    """Binary mask of the dark tube/emitter (dark pixels -> 255)."""
    _, m = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY_INV)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return m


def detect_tube(frame, vert_tol_deg=30, canny=(50, 150)):
    """Find the drip line and return a steering correction.

    Returns dict:
      found       — was a tube line found
      correction  — -5..+5 steering hint (+ = tube is RIGHT of center -> steer right)
      offset_px   — tube_x - frame_center_x (+ = right)
      tube_x      — estimated tube column (px)
      angle_deg   — median line angle
    """
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, canny[0], canny[1])
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50,
                            minLineLength=int(h * 0.30), maxLineGap=30)

    none = {"found": False, "correction": 0.0, "offset_px": None,
            "tube_x": None, "angle_deg": None}
    if lines is None:
        return none

    xs, angs = [], []
    for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):   # (N,1,4) or (N,4)
        ang = np.degrees(np.arctan2(y2 - y1, x2 - x1))     # -180..180
        if abs(abs(ang) - 90) < vert_tol_deg:              # near-vertical only
            xs.append((x1 + x2) / 2.0)
            angs.append(ang)
    if not xs:
        return none

    tube_x = float(np.median(xs))                          # robust to stray edges
    offset = tube_x - w / 2.0                              # + = tube right of center
    correction = float(np.clip(offset / (w / 2.0) * 5.0, -5, 5))
    return {"found": True, "correction": round(correction, 2),
            "offset_px": round(offset, 1), "tube_x": round(tube_x, 1),
            "angle_deg": round(float(np.median(angs)), 1)}


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
