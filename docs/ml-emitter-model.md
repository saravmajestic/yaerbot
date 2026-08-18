# ML Emitter Model — capture → train → deploy

The **single** reference for the Edge Impulse FOMO emitter detector: how frames are
collected, labelled, trained, and how the model actually reaches the robot. Everything
below is verified on hardware unless marked otherwise — most of it was learned the hard
way on 2026-08-15/16.

> **Why this model exists:** it turns emitter detection from classical CV into an
> on-device *learned* model — the strongest "Physical AI" claim for the contest.
> `vision/detect_emitter` (Canny/Hough width-bump) stays as the automatic fallback, so
> nothing is blocked if the model underperforms.

---

## 0. The pipeline at a glance

```
ESP32-CAM ──MJPEG──> UNO Q Linux ──> _cam_loop ──> detect_emitter_ml()
                                                        │  (falls back to
                                                        │   classical CV)
                                        HTTP :1337 ─────┘
                                        ei-obj-detection-runner container
                                        running your .eim
```

The model does **not** load in-process. The `object_detection` brick posts JPEGs to a
**separate container** over HTTP on port 1337. That container is started by App Lab
because the app declares the brick.

---

## 1. Capture

**Camera tab → "▶ Follow tube & capture"** (`scan` run mode). It creeps along the drip
line via classical CV and saves raw frames; **the seeder is never touched**. Capture
switches on automatically at start and off at Stop.

Or capture without driving: **Camera tab → Dataset capture → Start** (interval slider).

| | |
|---|---|
| Saved to | `~/motor-control/captures/cap_YYYYMMDD_HHMMSS_mmm.jpg` on the robot |
| Format | full-res **raw, unannotated** JPEG (~16–20 KB at 320×240) |
| Persistence | `/app/captures` is bind-mounted to the host — survives container restarts |
| Video | **none.** Stills only; there is no `VideoWriter` anywhere |

Pull them off and clear the board:

```bash
scp "unoq:/home/arduino/motor-control/captures/*.jpg" apps/farm-robot/captures/
ssh unoq 'rm -f /home/arduino/motor-control/captures/cap_*.jpg'
```

> `captures/` is **gitignored** — a real session runs to thousands of frames.
> Always hash both sides before deleting from the board.

### Capture gotchas

- **The UI count badge is an in-memory counter, not a directory scan.** It resets to 0
  on container restart and will not notice a manual `rm`. Trust `ls | wc -l`.
- **Watch the count, not the clock.** If it stops climbing the stream has died and you
  are recording nothing. At a 2 s interval it should rise ~30/minute.
- The ESP32-CAM link is the limiting factor, not disk (15 GB free ≈ 250k frames). Its
  firmware has **no WiFi rejoin loop** until `esp32_cam.ino` is flashed, so a dropped
  association is permanent until power-cycled.
- **Match capture conditions to demo conditions** — same time of day, sun angle, soil
  moisture. A model this narrow will not generalise, which is a fine trade for a single
  take, but only if the take looks like the training data.

---

## 2. Prune and label

Upload the folder to **Edge Impulse Studio** (web — nothing to install; the CLI is
optional). Settings that matter:

- Upload mode **folder**, image label format **Unlabeled**, label **Leave data unlabeled**
- **Automatically split** train/test

Then **Data acquisition → Labelling queue** and draw boxes.

| Rule | Why |
|---|---|
| Class name **`emitter`**, lowercase | must be in `EMITTER_LABELS` in `vision/emitter_ml.py`, else every detection is filtered out |
| **ONE box per emitter**, covering both holes | the pipeline consumes a single `position` per detection and plants a cross around it. FOMO predicts centroids on a grid ⅛ the input size, so two holes millimetres apart land in the same cell and cannot be separated anyway |
| **Label every emitter in every frame** | an unlabelled emitter teaches the model that emitters are background |
| **Keep** clean no-emitter frames, unlabelled | useful negatives |
| **Keep mildly motion-blurred frames** | ⚠️ the robot detects emitters **while moving** — it only stops once one is found. Creep-speed blur is the *operating condition*, so a model trained only on sharp frames fails exactly when it matters. Train on what you will infer on. |
| **Drop** only frames too blurred for *you* to locate the emitter, and frames where the tube has left the frame | genuinely unusable, and the tube-off-frame ones are where the follower was already failing |

Consecutive frames are near-duplicates, so Studio's **object tracking** in the labelling
queue propagates a box forward — 170 manual labels becomes ~20 corrections.

> **What matters is the number of labelled emitter instances, not the file count.**
> Under ~50 and no amount of tuning will substitute for another capture pass.

---

## 3. Train

**Create impulse** → Image block + **Object Detection (Images)** → **FOMO**.

**Set the image size to 160×160, not the 96×96 default.** The emitter is ~5–8 px in a
320-wide frame; at 96×96 it shrinks to ~2 px against a grid whose cells are 8 input px
each, and it trains badly.

### Reading the metrics

Both numbers mislead, in opposite directions:

- **FOMO's F1 reads low.** It scores on centroid matching — a near-miss counts as a
  false positive *and* a false negative. 66.7% F1 corresponded to **99.3% confidence**
  on a real frame when tested directly against the runner.
- **The test accuracy reads high.** The random train/test split leaks: consecutive
  frames 2 s apart are near-duplicates, so test frames are effectively training frames.

**Judge the model on the live annotated view, not on either number.** If you want an
honest figure, upload one drive as Training and a *separate* drive as Test.

Check whether errors are FN or FP — they mean different things: mostly-FN is fixable by
lowering `_EMIT_CONF`; mostly-FP is made *worse* by it.

### The bar to beat — measured baseline, 2026-08-18

Scored with `scripts/score_emitter_model.py` against the 77 USB frames in `captures/usb/`
(62 dataset frames + 15 detector-failure frames), probing at conf ≥ 0.05 to see everything
the model proposes rather than only what clears our gate:

| | first trained model |
|---|---|
| frames scored | 77 |
| frames with **any** box | **56 (73%)** |
| confidence min / median / max | 0.603 / **0.916** / 0.997 |
| frames clearing 0.50 / 0.70 / 0.90 | 56 / 49 / **29** |

`cap_20260818_040718_843.jpg` scored **0.997** and human inspection of that frame found
**plain drip tube and no emitter**. A high fire-rate with a high median and no gap in the
distribution is the signature of a model answering *"tube"* rather than *"emitter"* — and it
means **no value of `_EMIT_CONF` can separate them**. Do not spend time tuning the threshold
against a model in this state; the fix is negatives in the training set.

A retrain has worked when the plain-tube frames go QUIET while the labelled emitters still
fire. It has **not** worked if everything merely scores lower — that just slides the overlap.

> The emitters on this tube are **small dark punched holes in the tube wall, ~4-6 px across**
> at 320x240, and there are commonly one or two per frame. They are only clearly visible when
> zoomed; at 100% you will miss them, and a missed emitter in a saved frame actively teaches
> the model that emitters are background. **Zoom in while labelling.**

---

## 4. Deploy

**Deployment target: `Arduino UNO Q`** — *"An EIM binary for the Arduino UNO Q CPU"*.

- ❌ **not** `Arduino VENTUNO Q` — a different board, with Qualcomm NN accelerators
- ❌ **not** `C++ library (Linux)` — source you would have to compile yourself
- ✅ Choose **float32 (unoptimized)**, not the quantized build. Quantizing saves RAM
  (393 KB vs 1.5 MB), which matters on a microcontroller and **not at all** on a 4 GB
  Linux board. Don't trade accuracy for a resource you have in abundance.

Also set the **header target** away from the `Cortex-M4F 80MHz` default — it drives the
RAM/latency estimates, and MCU figures are meaningless for a model running on the
Qualcomm Linux side. Changing it re-computes estimates only; **it does not invalidate
your data, labels or trained weights.**

### Getting the model onto the robot

**The supported path is App Lab, and App Lab requires the USB-C cable.**

```
Connect UNO Q by USB-C (direct, not through a hub)
  → open the "Farm Robot Control" app          ← per-app, not a global setting
  → Bricks → Object Detection → AI Models
  → link the Edge Impulse account, select the project
  → install to board
```

**Network connection is not a substitute.** Verified on the board:

| | |
|---|---|
| App Lab daemon `:8800` | bound to **127.0.0.1 only** |
| Only network-facing port | SSH (22) |
| mDNS advertises `_arduino._tcp` | on port 80, where **nothing listens** |
| `arduino-router-serial.service` | proxies to `ttyGS0` — a **USB** gadget serial |

Over the network App Lab shows the brick catalogue and the cloud Edge Impulse link (so
your project appears), but lists **no apps** and the model Download fails with no trace
in the board's logs — because there is no board context to install into. Nothing is
broken; it is USB-transport by design.

**Do not use "import from computer"** — it creates a *new* app and can clobber the
running `motor-control`.

> The `/v1/models` registry on the board is **read-only and cloud-backed** (`Allow: GET,
> HEAD`; the model ids are not in the binary or any local file). There is no local file
> to edit to register a custom model, and `app-compose-overrides.yaml` is **regenerated
> on every app start**, so hand-editing it is silently reverted.

### Why `object_detection` and NOT `video_object_detection`

App Lab also offers `arduino:video_object_detection`. **Do not use it here.** Its
`brick_config.yaml` declares `required_devices: [camera]` and its runner is started with
`--mode streaming --camera /dev/video1`:

- our camera is an **ESP32-CAM over WiFi MJPEG** — there is no local `/dev/video*` to give it
- it would **own the stream** and emit its own annotated video, bypassing `_cam_loop`,
  which is deliberately the *sole consumer*: the same frame feeds tube-steering, the
  emitter detector, dataset capture and the browser overlay
- `object_detection` takes **image bytes**, which is exactly what we have

> **"Video" does not mean de-blurred.** That brick runs the same model on the same
> individual frames — the word describes input plumbing and streaming output, not
> temporal processing. Motion blur is optical (exposure × speed); the fixes are more
> light, slower motion, or a shorter shutter — never a different brick.

### Prerequisite: the brick must be declared

`console/app.yaml` must list the brick, or the runner container never starts:

```yaml
bricks:
  - arduino:web_ui
  - arduino:object_detection      # requires_container: true -> starts ei-obj-detection-runner
```

---

## 5. Enable and verify

**The ML path is OFF by default.** `emitter_ml.py` gates the whole brick import behind
an env var, because constructing the brick with no model blocks ~60 s trying to reach
the runner:

```bash
FARMOS_EMITTER_ML=1
```

Without it `ml_available()` stays `False` and the console silently uses classical CV.

Restart the app, then confirm from the camera-loop log line:

```
camera loop (sole consumer): ...  [emitter: ML/Edge-Impulse]   <- ML live
camera loop (sole consumer): ...  [emitter: classical CV]      <- still fallback
```

Confirm the runner has **your** model, not the OOTB `yolo-x-nano`:

```bash
ssh unoq 'curl -s http://127.0.0.1:1337/api/info' | head -20
# want: project.name = your project, labels = ['emitter'], input 160x160,
#       model_type = constrained_object_detection      (NOT 80 COCO classes)
```

---

## 6. Gotchas that cost real time

### The detect() response shape — three traps

`ObjectDetection.detect()` does **not** return the raw Edge Impulse JSON. Verified on
the board:

```python
# what the brick returns
{"detection": [{"class_name": "emitter",
                "confidence": "99.33",                 # STRING, 0..100
                "bounding_box_xyxy": [x1, y1, x2, y2]}]}

# what the raw runner returns on :1337/api/image
{"result": {"bounding_boxes": [{"label": "emitter", "value": 0.993,
                                "x": 148, "y": 108, "width": 12, "height": 36}]}}
```

1. the key is **`detection`** — singular, not `detections`
2. the label key is **`class_name`**, not `label`
3. confidence is a **percentage string**, not a 0–1 float

`_extract_boxes()` handles both shapes now. It originally guessed and matched neither,
which would have produced a model that loads, logs `[emitter: ML/Edge-Impulse]`, and
detects nothing on every frame. Had only the confidence been wrong, `"99.33"` would have
been read as 99.3 instead of 0.993 and sailed past every threshold.

### The confidence threshold trap

Two thresholds stack, and the arithmetic bites on dry soil:

```
fused = 0.6 * ml_value + (0.4 if wet else 0.0)      # detect_emitter_ml
plant if fused >= _EMIT_CONF (0.55)                 # console
```

A **visual-only** detection maxes at **0.6**. So a 0.9-confidence emitter on dry soil
scores **0.54 and does not trigger a plant.** If the annotated view shows clean
detections but nothing plants, this is why — not the model.

### Fallback workaround (no USB)

Your `.eim` runs in the same container image App Lab uses, so the brick can be pointed
at it manually:

```bash
docker run -d --name eim-test -p 127.0.0.1:1338:1338 \
  -v /home/arduino/.arduino-bricks/ei-models:/models-custom \
  ghcr.io/arduino/app-bricks/ei-models-runner:0.5.0 \
  --model-file /models-custom/emitter-fomo.eim --run-http-server 1338
curl -s -F "file=@frame.jpg;type=image/jpeg" http://127.0.0.1:1338/api/image
```

This is how the model was validated without App Lab. Replacing App Lab's runner the same
way works, but the compose overrides are regenerated on every app start, so it must be
re-applied after each restart. Treat it as a demo-day fallback, not the normal path.

---

## References

- Custom AI models in App Lab — https://docs.arduino.cc/software/app-lab/integrations/ai-models/
- App Lab setup (macOS) — https://docs.arduino.cc/software/app-lab/setup/macos/
- Deploying a `.eim` on the UNO Q — https://forum.arduino.cc/t/how-to-deploy-eim-model-on-arduino-uno-q/1452107
- Code: `vision/emitter_ml.py`, `console/python/main.py` (`_cam_loop`), `console/app.yaml`
