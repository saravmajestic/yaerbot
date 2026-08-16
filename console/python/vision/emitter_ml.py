"""On-device ML emitter detector — the UNO Q "Physical AI" showcase.

Runs a trained Edge Impulse object-detection model (FOMO is ideal for small
objects like a ~3mm emitter) through the Arduino App Lab `object_detection`
brick, and returns the SAME dict shape as vision.detect_emitter() so it drops
straight into the existing frame -> detect -> fuse-with-moisture -> plant loop.

Design notes:
- Fully OPTIONAL and self-guarding. If the brick isn't importable (e.g. running
  off-device / offline tests) or no model is deployed yet, `ml_available()`
  returns False and the caller falls back to the classical vision.detect_emitter.
- The classical detector stays as the Plan-B fallback — try ML first, fall back.

FULL WORKFLOW — capture, label, train, deploy, and the gotchas that cost real time —
is in docs/farm-os/ml-emitter-model.md. Do not duplicate those steps here.

Short version: the model is installed by App Lab (which needs the USB cable), the brick
must be declared in app.yaml, and this module stays dormant until FARMOS_EMITTER_ML=1.
"""
import os
import cv2

# Class name(s) the trained model uses for a drip emitter. Update to match the
# label you use in Edge Impulse (the model may also emit a background class).
EMITTER_LABELS = {"emitter", "dripper", "hole"}

# OFF by default: constructing the object_detection brick with no deployed model
# blocks ~60s trying to reach the EI runner service. Enable only once a model is
# deployed, via env: FARMOS_EMITTER_ML=1
_ENABLED = os.environ.get("FARMOS_EMITTER_ML", "").strip().lower() in ("1", "true", "yes", "on")

_BRICK_OK = False
if _ENABLED:
    try:
        from arduino.app_bricks.object_detection import ObjectDetection
        _BRICK_OK = True
    except Exception:                            # noqa: BLE001 (off-device / offline)
        ObjectDetection = None
else:
    ObjectDetection = None

_od = None
_model_ready = None          # None = not checked yet, True/False after first probe


def _ensure():
    """Lazily build the brick and probe for a deployed model (once)."""
    global _od, _model_ready
    if not _BRICK_OK:
        _model_ready = False
        return None
    if _od is None:
        try:
            _od = ObjectDetection(confidence=0.3)
        except Exception:                        # noqa: BLE001
            _model_ready = False
            return None
    if _model_ready is None:
        try:
            info = _od.get_model_info()
            _model_ready = info is not None
        except Exception:                        # noqa: BLE001
            _model_ready = False
    return _od if _model_ready else None


def ml_available():
    """True only when the brick is importable AND a model is actually deployed."""
    _ensure()
    return bool(_model_ready)


def _extract_boxes(det):
    """Normalise the brick's detect() result to a list of {label, value, cx, cy}.

    VERIFIED against the real brick on the board 2026-08-16 — the earlier guesses here
    were all wrong and would have returned [] on every frame: a model running perfectly
    while the robot detected nothing. ObjectDetection._extract_detection() returns:

        {"detection": [{"class_name": "emitter",
                        "confidence": "99.33",               <- STRING, 0..100
                        "bounding_box_xyxy": [x1, y1, x2, y2]}]}

    Note the three traps: the key is "detection" (singular, not "detections"), the
    label key is "class_name" (not "label"), and confidence is a percentage STRING,
    not a 0..1 float — feeding it straight through would have made every detection
    read as ~99.0 and sail past every threshold.

    The raw runner shape ({"result": {"bounding_boxes": [...]}}, label/value/x/y/w/h)
    is also handled, for anyone talking to the EI HTTP server directly.
    """
    if not det:
        return []
    out = []

    # --- brick shape: {"detection": [{class_name, confidence "0..100" str, xyxy}]}
    if isinstance(det, dict) and isinstance(det.get("detection"), list):
        for b in det["detection"]:
            try:
                xy = b.get("bounding_box_xyxy") or [0, 0, 0, 0]
                out.append({
                    "label": str(b.get("class_name", "")),
                    "value": float(b.get("confidence", 0.0)) / 100.0,   # -> 0..1
                    "cx": int((float(xy[0]) + float(xy[2])) / 2),
                    "cy": int((float(xy[1]) + float(xy[3])) / 2),
                })
            except Exception:                    # noqa: BLE001
                continue
        return out

    # --- raw EI runner shape: {"result": {"bounding_boxes": [...]}} (or unwrapped)
    boxes = None
    if isinstance(det, dict):
        res = det.get("result")
        if isinstance(res, dict):
            boxes = res.get("bounding_boxes")
        if boxes is None:
            boxes = (det.get("bounding_boxes") or det.get("detections")
                     or det.get("results"))
    elif isinstance(det, list):
        boxes = det
    for b in boxes or []:
        try:
            label = str(b.get("label", b.get("class", "")))
            value = float(b.get("value", b.get("confidence", 0.0)))
            x = float(b.get("x", 0)); y = float(b.get("y", 0))
            w = float(b.get("width", b.get("w", 0)))
            h = float(b.get("height", b.get("h", 0)))
            out.append({"label": label, "value": value,
                        "cx": int(x + w / 2), "cy": int(y + h / 2)})
        except Exception:                        # noqa: BLE001
            continue
    return out


def detect_emitter_ml(frame, moisture=None, moisture_wet_below=9000, conf_min=0.3):
    """ML emitter detection, fused with moisture. Returns the same shape as
    vision.detect_emitter plus `source`/`ml_ready`. When `ml_ready` is False the
    caller should fall back to the classical detector."""
    result = {"detected": False, "position": None, "confidence": 0.0,
              "visual": False, "wet": False, "source": "ml", "ml_ready": False}
    od = _ensure()
    if od is None:
        return result                            # no model -> caller falls back
    result["ml_ready"] = True

    ok, buf = cv2.imencode(".jpg", frame)
    if not ok:
        return result
    try:
        det = od.detect(buf.tobytes(), "jpg", confidence=conf_min)
    except Exception:                            # noqa: BLE001
        result["ml_ready"] = False               # inference failed -> fall back
        return result

    boxes = [b for b in _extract_boxes(det)
             if not EMITTER_LABELS or b["label"].lower() in EMITTER_LABELS]
    wet = moisture is not None and moisture < moisture_wet_below
    result["wet"] = bool(wet)
    if not boxes:
        return result

    best = max(boxes, key=lambda b: b["value"])
    result.update(visual=True, detected=True,
                  position=(best["cx"], best["cy"]))
    # confidence: ML visual primary, moisture confirms (mirrors detect_emitter)
    result["confidence"] = round(0.6 * min(1.0, best["value"]) + (0.4 if wet else 0.0), 2)
    return result
