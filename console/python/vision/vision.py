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
    # FULL WIDTH AT HALF MAX, always computed — this is the one that measures the FEATURE.
    # Verified against the 16mm tube as a ruler (it reads 13-21px across in these frames), so
    # FWHM is the physically meaningful number and it is now reported unconditionally.
    half = mag[peak] * 0.5
    l = peak
    while l > 0 and mag[l - 1] > half:
        l -= 1
    r = peak
    while r < n - 1 and mag[r + 1] > half:
        r += 1
    fwhm_centre, fwhm = (l + r) / 2.0, r - l + 1

    centre = width = None
    paired = False
    if partner_win.max() > 1.2 * sigma:
        c, wd = (peak + partner) / 2.0, abs(partner - peak) * 2
        if min_w <= wd <= max_w:
            # PAIR-CENTRING MUST NOT LEAVE THE SEED WINDOW. The window says where the tube may be;
            # the peak is chosen inside it, but the midpoint of a peak/partner pair can sit well
            # outside — and nothing used to re-check that.
            #
            # Found 2026-08-19 on cap_20260819_094944: the caller asked for x=185 +/-20, i.e.
            # 165..205, and this returned 147.5 — 38 px OUTSIDE the window, sitting on a dark soil
            # region. Two of four bands were dragged off the tube that way, the line fit needs three,
            # and the frame was rejected `line-fit-2-of-3` with the tube plainly visible. The robot
            # stopped. Tightening the window could not help: at +/-20 the answer was still 147, and
            # at +/-15 the band found nothing at all.
            #
            # The peak was on the tube. Only the pairing moved it. So when the pair centre leaves the
            # window, keep the peak instead of reporting a position the caller explicitly excluded.
            if seed is None or seed_win is None or abs(c - seed) <= seed_win:
                centre, width, paired = c, wd, True

    if centre is None:                             # no partner, or its centre left the window
        centre, width = fwhm_centre, fwhm

    if not (min_w <= width <= max_w):
        return None
    # `width` IS NOT A PHYSICAL WIDTH AND MUST NOT BE GATED ON. It is |partner-peak|*2 when
    # pair-centring succeeded and the FWHM when it did not, so the SAME tube at the SAME
    # distance reports ~44-64px or ~12-20px depending purely on which branch ran. Measured
    # over 2086 consecutive frames: min 12, p10 14, median 44, p90 64 — bimodal, and the two
    # modes are the two code paths, not two tube sizes.
    #
    # That cost a whole field session on 2026-08-18: _TUBE_MIN_W_PX = 30 in main.py was
    # calibrated on pair-centred frames only, so it was silently gating on "did pair-centring
    # succeed" rather than "is this plausibly a tube". Frames with the tube plainly visible and
    # all four bands correctly on it, at 4.2-5.9 sigma, were thrown away for reporting w=12.
    # `width_fwhm` is the number to gate on; `paired` says which branch produced `width`.
    return {"pos": float(centre), "width": int(width),
            "width_fwhm": int(fwhm), "paired": paired,
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
#
# CHANGED 4 -> 3 on 2026-08-18, on 2086 CONSECUTIVE frames (26.3fps, one unbroken 81-second
# pass) instead of the 20 scattered frames the old value was chosen on. The old measurement was
# not wrong about its own frames; it was measured through main.py's width gate, which turned
# out to be broken, so both numbers were confounded.
#
#   bands  gate                recall   longest blind run   halts   dx_med  dx_p95
#     4    width >= 30           33%        6.57s            37      2.0    18.0
#     3    width_fwhm >= 5       87%        1.10s             9      2.0    36.2
#     3    (no width gate)       91%        0.46s             7      2.0    34.5
#
# Rejections at 4 bands were dominated by `line-fit-3-of-4`: 863 frames — 41% OF THE WHOLE PASS
# — had exactly three of four bands on a line and were thrown away. The tube is visible in them
# and the bands are on it.
#
# WHY THIS IS SAFE WITHOUT LABELS. dx_med (the tube's apparent movement between consecutive
# accepted frames) stays at 2.0px in every configuration above. If relaxing the gate were
# admitting soil and shadow, the accepted positions would scatter and that number would climb.
# It does not — the extra detections are temporally coherent, which is what a real tube looks
# like and what a false positive cannot fake. The p95 tail DOES grow, 18 -> 36px, and that is
# precisely what _track_tube's jump gate (~30px at 30fps) exists to catch: relax the per-frame
# detector, let the temporal gate handle the tail.
#
# WHAT THIS MEASUREMENT CANNOT SAY. The pass contains no non-tube scenes, so it does not
# measure the false-positive rate on bare soil — which is what justified 4 bands originally
# (4 false positives in 11 non-tube frames). Two things bound that risk now: `detect_crossing`
# does NOT use _MIN_BANDS at all, so the traverse latch that decides a row change is unaffected;
# and the remaining consumer is _turn_onto_tube's alignment loop, which is capped at 6 nudges
# and hands over to the follow loop regardless.
#
# REVERTED TO 4 on 2026-08-18, after the first field run with 3 drove the robot off the row.
#
# THE FRAME THAT SETTLED IT — cap_20260818_104428_437.jpg, the FIRST frame of the run:
#     bands_raw = [135.5, 268.0, 278.5, 293.5]
#     _MIN_BANDS=4 -> found=False, reject=line-fit-3-of-4   (correctly refuses)
#     _MIN_BANDS=3 -> found=True,  x_near=293.5, corr=+4.17 (confidently wrong)
# The tube is plainly in frame, running top-centre to bottom-left, and the band at 135.5 is
# ON it. The other three sit on the edge of an OVEREXPOSED white patch on the right. The
# 3-of-4 fit picked the wrong three, the robot steered hard right, and every correction for
# the rest of the run stayed positive because it was then following the overexposure boundary.
#
# WHY THE 2086-FRAME MEASUREMENT DID NOT CATCH THIS. It is a real measurement and its numbers
# stand — 33% -> 84% recall on that pass, with the tube position bit-identical where both
# settings fired. But that pass contained NO COMPETING near-vertical feature: no blown
# highlights, tube large and central throughout. The caveat was written down at the time and
# then under-weighted. A 3-of-4 fit is a weak geometric constraint, and the angle gate does not
# save it when the competitor is ALSO near-vertical.
#
# WHAT WOULD EARN 3 BACK, in order:
#   1. FIX THE EXPOSURE FIRST. The competing feature is a blown highlight — measured
#      saturation 16 and value 180 in that column, against 38/122 for real tube. It exists
#      because auto-exposure is running with gain up at 131. scripts/fix_camera.sh --pin
#      removes it at source, which is better than teaching the detector to ignore it.
#   2. Then re-measure on a pass that CONTAINS competing features, not one that lacks them.
#   3. If 3 is still wanted, it needs a tie-break the current fit does not have: prefer the
#      hypothesis nearest the last ACCEPTED tube column. _track keeps that value, but it lives
#      in main.py and the detector never sees it. That is a real change, not a constant.
#
# Cost of being at 4, honestly: far more `line-fit-3-of-4` rejections and more halts. A halt is
# recoverable — the run stays active and resumes when the tube is seen. Steering off the row is
# not.
_MIN_BANDS = 3
#
# FINAL VALUE FOR TODAY: 3. Set to 3 on an offline measurement, reverted to 4 after one field
# failure, and back to 3 after the field showed 4 CANNOT FOLLOW THE TUBE AT ALL.
#
# The deciding run, 12:15-12:16, 36 seconds at _MIN_BANDS=4:
#     travelled 0.23m  -> FROZEN across all 8 log windows
#     rejects: 97 per 5s, line-fit-3-of-4 = 100%
#     tube held 0% of frames
# The robot drove 23cm on its last good reading and then stopped for good. And the frames it was
# rejecting are not marginal — lost_121524.jpg has the tube dead centre, obvious, with ALL FOUR
# bands on it at [178.0, 173.5, 197.0, 159.0].
#
# WHY 4 FAILS: those four numbers span 38px across a tube only ~35px wide. The bands land at
# different points across the tube's OWN width — some on an edge, some on the centre — and no
# line fits all four within max_line_dev_px. It is not a detection failure; it is per-band
# position noise being asked to lie on a line more precisely than it can.
#
# THINGS THAT DID NOT FIX IT, all measured rather than assumed:
#   * max_line_dev_px 22 -> 40: recovers 1 extra frame of 4, dense recall 55% -> 60%. Not enough.
#   * adding RANSAC's missing refit step (least squares on the best hypothesis' inliers, then
#     re-count): 0 of 3 recovered. The refit pulls toward the 3 inliers it started from, so it
#     never reaches the line that fits all four.
#
# THE COST OF 3, honestly: on 2026-08-18 at 10:44 it took the wrong three of four bands —
# [135.5, 268.0, 278.5, 293.5], one on the tube and three on an overexposed edge — and steered
# off the row. Note what that frame was: THE FIRST FRAME OF A RUN, with the tube already at the
# frame edge and no previous position to sanity-check against. In steady following _track_tube's
# jump gate (~30px) rejects a 158px leap like that outright; it could only happen with no anchor.
#
# SO THE OPERATIONAL RULE IS: START THE RUN WITH THE TUBE NEAR FRAME CENTRE. That removes the
# only observed failure of 3, and it is what an operator would do anyway.
#
# THE REAL FIX, still not built: pass the last accepted tube column into detect_tube and score
# candidate fits by (inliers, -distance from it) instead of inliers alone. _track["x"] holds it;
# the detector never sees it. That accepts today's 12:16 frames AND rejects the 10:44 one, with
# one setting and no operational rule. ~20 lines, and it is the thing to do before trusting this
# for a demo run.

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
                max_line_dev_px=22, max_angle_deg=35.0,
                hint_x=None, hint_win=45, **kw):
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
    # EVERY REJECTION SAYS WHICH STAGE REJECTED IT. Returning a bare found=False forced the
    # operator's log to read "tube lost" and left the reason to be reconstructed from numbers
    # printed beside it — which is how a width gate rejecting 4.2-sigma detections of a plainly
    # visible tube went unnoticed through a whole field session. The caller aggregates these
    # into a histogram, so "94% of rejections were `width`" is visible in one run.
    none = {"found": False, "correction": 0.0, "offset_px": None, "tube_x": None,
            "angle_deg": None, "width": None, "width_fwhm": None, "paired": False,
            "strength": 0.0, "polarity": None, "reject": "no-profile"}
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

    # SEARCH NEAR WHERE THE TUBE WAS, THEN ANYWHERE.
    #
    # This is the fix for the failure that no value of _MIN_BANDS could avoid. Each band
    # independently takes the strongest profile peak across the WHOLE frame width, so a band
    # will happily land on a shadow edge, a straw patch or a sunlit boundary instead of the
    # tube — and then the line fit either rejects the frame (4-of-4) or draws a line through
    # the wrong points (3-of-4). Both were observed in the field on 2026-08-18:
    #
    #   10:44  bands [135.5, 268.0, 278.5, 293.5]  one on the tube, three on an overexposed
    #                                              edge -> reported x=293, steered off the row
    #   12:16  bands [178.0, 173.5, 197.0, 159.0]  ALL on the tube, 38px scatter across a 35px
    #                                              tube -> no line fits 4, robot froze for 36s
    #   12:4x  bands [185.5, 156.5, 171.0,  70.5]  bottom band 70px off the tube -> steep wrong
    #                                              line, x=70 where the truth was ~140
    #
    # `hint_x` is the last ACCEPTED tube column from the caller's tracker. Seeded, each band
    # searches only within hint_win of it, so a band cannot run off to an unrelated feature and
    # the scatter that broke the fit collapses. If the seeded attempt does not produce a usable
    # fit — the tube genuinely moved, or we never had it — it falls back to the unseeded search
    # and re-acquires from scratch. Tracking when it can, re-acquiring when it must.
    #
    # NOTE the existing warning in the two-pass comment above still stands and is why this is
    # NOT seeded band-to-band within a frame: forcing continuity onto noise manufactures the
    # continuity the test looks for. Seeding from the PREVIOUS FRAME is different — it is an
    # independent measurement of where the tube was, not a self-referential one.
    def _scan(seed):
        out = []
        for i in range(bands):
            r = _profile_line(gray[i * step:(i + 1) * step, :], 0,
                              seed=seed, seed_win=(hint_win if seed is not None else None),
                              **kw)
            out.append(None if r is None else r["pos"])
        return out

    def _n_inliers(cand):
        pl = [(ys_all[i], cand[i]) for i in range(bands) if cand[i] is not None]
        if len(pl) < _MIN_BANDS:
            return 0
        top = 0
        for a in range(len(pl)):
            for b in range(a + 1, len(pl)):
                (y1, x1), (y2, x2) = pl[a], pl[b]
                if y2 == y1:
                    continue
                m = (x2 - x1) / float(y2 - y1)
                c = x1 - m * y1
                top = max(top, sum(1 for y, x in pl
                                   if abs(x - (m * y + c)) <= max_line_dev_px))
        return top

    # WHEN HINTED, DO NOT FALL BACK TO A FULL SEARCH. Measured on live_1245.png: with the hint
    # 30px low the seeded scan fails, and the full-search fallback then returns x=70 — the bad
    # band, 80px off the tube — where reporting nothing would have been correct. A wrong answer
    # is worse than no answer here, because the steering acts on it.
    #
    # Re-acquisition is not lost: the caller stops hinting once its anchor goes stale (see the
    # hint expiry in main.py), and then this runs unseeded and re-acquires from scratch.
    seeded_used = False
    if hint_x is not None:
        indep = _scan(float(hint_x))
        seeded_used = True
    else:
        indep = _scan(None)

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
        # Measured over 2086 consecutive frames: this fires on 3% of them. The bands almost
        # always FIND something — it is agreement, below, that is hard.
        return dict(none, reject="bands-found-%d" % len(pts),
                    width=whole["width"], width_fwhm=whole["width_fwhm"],
                    strength=whole["strength"], polarity=whole["polarity"])

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
        return dict(none,
                    reject="line-fit-%d-of-%d" % (0 if best is None else best[0][0], len(pts)),
                    bands_raw=[None if v is None else round(v, 1) for v in indep],
                    width=whole["width"], width_fwhm=whole["width_fwhm"],
                    strength=whole["strength"], polarity=whole["polarity"])
    n_inliers = best[0][0]
    _, m, c = best

    ys = list(ys_all)
    xs = [m * y + c for y in ys]              # the LINE's value at each band height

    x_far, x_near = xs[0], xs[-1]             # top of frame = furthest ahead
    # + = the tube leans right going DOWN the frame. Reported for logging; the steering
    # uses far and near offsets directly, which carry the same sign convention as
    # `correction` and so cannot be got backwards.
    angle = round(float(np.degrees(np.arctan2(x_near - x_far, ys[-1] - ys[0]))), 1)

    # THE ROW RUNS TOP-TO-BOTTOM. This gate replaces the axis-dominance test removed on
    # 2026-08-18, and it is the constraint that makes _MIN_BANDS=3 safe.
    #
    # Why the replacement was needed: a 3-of-4 robust fit is a WEAK geometric constraint. With
    # four scattered band positions there are 12 chances (6 pairs x 2 remaining points) for some
    # pair's line to pass within max_line_dev_px of a third, so it accepts pure scatter most of
    # the time. Measured on a synthetic HORIZONTAL line — the one thing detect_tube must never
    # claim, or the robot turns onto the row it is already following — bands came back
    # [280.5, 219.5, 49.0, 117.0] and a 3-inlier fit "succeeded".
    #
    # Residual magnitude does NOT separate the two (chance fits 3.8-15.8px max residual, real
    # 3-inlier fits median 5.8 / p90 16.0 — complete overlap). ANGLE does, cleanly:
    #     real tube, 1919 accepted frames: p50 6.7  p75 12.9  p90 18.4  p95 23.6  p99 34.6 deg
    #     the horizontal negatives that got through: -42.2 and 51.8 deg
    # 35 deg keeps 99.1% of real frames and rejects both.
    #
    # Unlike the axis test this operates on the FITTED LINE, not on a whole-frame profile — so
    # it cannot fail the way that test did, where a diagonal tube smeared to 3.4 sigma and lost
    # to a crisp band of straw.
    if abs(angle) > max_angle_deg:
        return dict(none, reject="angle-%.0f>%.0f" % (abs(angle), max_angle_deg),
                    angle_deg=angle, bands_used=n_inliers,
                    bands_raw=[None if v is None else round(v, 1) for v in indep],
                    width=whole["width"], width_fwhm=whole["width_fwhm"],
                    strength=whole["strength"], polarity=whole["polarity"])

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
            "angle_deg": angle, "width": whole["width"], "seeded": seeded_used,
            # width_fwhm is the PHYSICAL width (see _profile_line) and the one to gate on;
            # `width` is bimodal because it changes meaning with `paired`.
            "width_fwhm": whole["width_fwhm"], "paired": whole["paired"],
            "reject": None,
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
