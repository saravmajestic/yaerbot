# Plain tube — the emitter model's false positives

`plaintube_falsepositive_annotated.png` is the frame that ended ten iterations of rewriting the
emitter counting logic. Captured 2026-08-19 with the robot parked at the 1.10 m mark of a dry run —
one of two stops that matched no real emitter.

**There is no emitter anywhere in it.** It is plain tube. The model reports THREE boxes, at
confidence **0.77, 0.97 and 0.94**, stacked along the pipe at 79, 72 and 64 cm ahead of the punch.

Overlay: green = the tube as `detect_tube` sees it, orange = the commit band (39–49 cm ahead of the
punch), magenta circles = every box `detect_emitter_ml` returned, labelled `confidence  distance`.

## Why it matters

Every counting scheme built on this signal was counting the tube itself, which is why the band was
occupied on 70–96% of frames and why the stop count ended up paced by whatever lockout was set
(0.06 m → duplicates 0.11–0.19 m apart; 0.28 m → commits every 0.28–0.32 m). The operator raised
"false positives on plain tube" days earlier and it was wrongly retracted after a frame was judged to
contain emitter holes it did not.

## What this fixture is for

1. **A regression check on the next retrain.** Run `detect_emitter_ml` on it: a model that has been
   taught plain tube should return NO boxes, or none above ~0.6. Today it returns three at up to 0.97.
2. **A reminder of the shape of the fault** — the boxes sit ON the tube, at high confidence, spread
   along its length. No downstream filter (on-tube, band, lockout) can separate that from a real
   emitter, because it *is* on the tube.

## Missing, and needed

A **raw** frame (the robot was switched off before one could be taken — this is the annotated 3x
upscale). And more importantly a proper plain-tube negative SET for the training data: long runs of
pipe with no emitter, including the bright specular streak visible here, which may be what the model
is actually firing on.
