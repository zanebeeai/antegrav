#!/usr/bin/env python3
"""Colour-threshold orange cone detection -- for controlled-course testing.

WHY: ethon_v1 (the 14-class TRT model) is weak on the small flat orange cones
this team actually uses -- measured 2026-08-18 on a real garage frame, its best
cone scored 0.376 against a 0.45 cutoff, and it missed the other five entirely.
Lowering the model's threshold is NOT a fix: it drags in person/cyclist false
positives at 0.18/0.07, and those classes feed the hazard logic, so the car
would brake at ghosts.

On a controlled course the cones are the only saturated-orange things in the
scene, which makes colour a far stronger cue than shape. No training, no
inference cost, and it does not care whether the cone is tall or flat.

Output is deliberately the SAME shape the model path produces -- the
bottom-centre pixel of each blob (the ground-contact point) -- so callers push
it through the identical undistort+homography projection and the ground maths
can never drift between the two paths.

TUNING: run this file directly against a saved frame to see and tune what it
picks up, before trusting it:

    python3 orange_cone.py /dev/shm/ethon_cam_narrow.jpg out.jpg

It prints every blob it kept (and why others were dropped) and writes an
annotated image. Retune HSV_LO/HSV_HI on the real course in the real light --
these bounds were set against an indoor garage frame.
"""

import cv2
import numpy as np

# ---- tunables ------------------------------------------------------------
# OpenCV hue is 0-179 (NOT 0-359). Traffic-cone orange sits ~5-22. Keeping the
# lower bound at 5 (not 0) is what rejects RED objects -- there is a red
# toolbox in the test frame that a 0-lower-bound grabs immediately. High
# saturation is the other half of the filter: it rejects beige/tan floor,
# cardboard and skin, which share orange's hue but are washed out.
HSV_LO = (5, 110, 70)
HSV_HI = (22, 255, 255)

# The VALUE (brightness) floor in HSV_LO is only the BRIGHT-SCENE reference --
# the actual floor is computed per frame, scaled to how bright the scene is.
#
# Why: a fixed floor of 70 is the wrong test in dim light. Measured symptom
# (2026-08-18, garage after dark): the cone nearest the light was detected and
# the rest were not -- they were still orange and still saturated, just DARKER
# than 70, so inRange dropped them before they could ever become blobs. That
# failure is invisible in the reject list, because a pixel excluded by the mask
# never becomes a candidate at all.
#
# Scaling to the scene median tracks the exposure instead of fighting it: a
# bright frame keeps roughly the old behaviour, a dark frame lowers the bar by
# the same proportion the whole image dropped. Clamped at both ends so a
# nearly-black frame cannot drop the floor into sensor noise, and a blown-out
# frame cannot raise it above the value that already worked.
V_FLOOR_REL = 0.55        # fraction of the frame's median V
V_FLOOR_MIN = 25          # never below this (noise floor)
V_FLOOR_MAX = 70          # never stricter than the old fixed floor

# Minimum blob size as a FRACTION of frame area, not absolute pixels.
#
# This must be fractional. The tuning trap (hit 2026-08-18): the annotated
# previews in /dev/shm are downscaled to ANNOT_MAX_W=800 (800x450), but
# detection runs on the full 1280x720 CSI frame -- a 2.56x area difference. An
# absolute threshold tuned by eye on a preview is therefore 2.56x more
# permissive in production, which is exactly what flooded the wide camera with
# tan-wood and cardboard false positives. A fraction transfers correctly
# between the preview, the CSI sources, and the perc-1 TCP sources, which are
# all different resolutions.
#
# 5e-4 was MEASURED on a real garage frame: on the 800x450 preview a sweep
# showed 180 px (= 5e-4 of that frame) keeps 5 real cones and only 2 clutter
# blobs, where 50 px kept 9 real but also 4 clutter plus 11 specks. Raising the
# SATURATION floor instead does NOT work -- at sat>=170 the cones vanish before
# the clutter does, because indoor cones are not strongly saturated.
# Range cost: at 5e-4 a cone is still detected well past the 2-6 m pure-pursuit
# lookahead. Lower it only if you need cones beyond ~10 m on a clean course.
MIN_AREA_FRAC = 5e-4
MAX_AREA_FRAC = 0.04      # bigger than this fraction of frame is not a cone
                          # (an orange tarp, a wall, the car's own bodywork)
MIN_FILL = 0.30           # blob area / bounding-box area. A cone is fairly
                          # solid; a thin orange cable or a ragged reflection
                          # is not. Loose on purpose -- flat cones seen
                          # obliquely are not neat rectangles.
MAX_ASPECT = 4.0          # w/h or h/w beyond this is a stripe, not a cone
MORPH_K = 3               # open/close kernel: removes single-pixel speckle
                          # without eating a 40x60 px cone
MAX_BLOBS = 40            # hard cap so a pathological frame cannot flood the
                          # planner with hundreds of phantom cones

# Ignore blobs whose BOTTOM edge sits above this fraction of frame height.
# A cone stands on the ground, so its base is in the lower part of the image;
# anything whose base is up near the horizon is on a wall, a shelf or the
# ceiling. This matters most on the 184-degree wide fisheye, which sees the
# whole room and found orange on shelving at y/h ~ 0.22 (2026-08-18).
# curb_detect.py draws the same line for the same reason (ROW_Y_FRAC).
# Raising this cuts maximum detection RANGE (distant cones sit higher in
# frame), so it is deliberately loose -- the post-projection ground gate in
# birdseye_fusion._process_orange does the precise rejection.
ROI_TOP_FRAC = 0.35

ANNOT_PREVIEW_W = 800     # matches birdseye_fusion.ANNOT_MAX_W; used only to
                          # warn when this file is run against a preview JPEG
                          # rather than a full-resolution frame


def value_floor(hsv):
    """Per-frame brightness floor, scaled to the scene. See V_FLOOR_REL."""
    med = float(np.median(hsv[:, :, 2]))
    return int(max(V_FLOOR_MIN, min(V_FLOOR_MAX, round(V_FLOOR_REL * med))))


def _mask(frame, with_floor=False):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    vf = value_floor(hsv)
    lo = np.array((HSV_LO[0], HSV_LO[1], vf), np.uint8)
    m = cv2.inRange(hsv, lo, np.array(HSV_HI, np.uint8))
    k = np.ones((MORPH_K, MORPH_K), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
    return (m, vf, hsv) if with_floor else m


def detect(frame, debug=False):
    """Orange blobs -> ground-contact pixels.

    Returns a list of (u, v) pixel coords: horizontal centre, BOTTOM edge of
    each blob -- the same ground-contact convention the model path uses
    ((x1+x2)/2, y2). Ordered nearest-first (largest v = lowest in frame).

    With debug=True returns (kept, rejected) where each entry carries the
    bounding box and the reason, for tuning.
    """
    h, w = frame.shape[:2]
    min_area = MIN_AREA_FRAC * h * w
    max_area = MAX_AREA_FRAC * h * w
    m = _mask(frame)
    n, _, stats, cent = cv2.connectedComponentsWithStats(m, connectivity=8)

    kept, rejected = [], []
    for i in range(1, n):                      # 0 is background
        x, y, bw, bh, area = (int(stats[i, cv2.CC_STAT_LEFT]),
                              int(stats[i, cv2.CC_STAT_TOP]),
                              int(stats[i, cv2.CC_STAT_WIDTH]),
                              int(stats[i, cv2.CC_STAT_HEIGHT]),
                              int(stats[i, cv2.CC_STAT_AREA]))
        box = (x, y, bw, bh, area)
        if area < min_area:
            rejected.append((box, "area<%.0f (min)" % min_area))
            continue
        if (y + bh) < ROI_TOP_FRAC * h:
            rejected.append((box, "base above ROI (wall/shelf)"))
            continue
        if area > max_area:
            rejected.append((box, "area>%.0f (too big)" % max_area))
            continue
        if bw <= 0 or bh <= 0:
            rejected.append((box, "degenerate"))
            continue
        if area / float(bw * bh) < MIN_FILL:
            rejected.append((box, "fill %.2f<%.2f" % (area / float(bw * bh),
                                                      MIN_FILL)))
            continue
        asp = max(bw / float(bh), bh / float(bw))
        if asp > MAX_ASPECT:
            rejected.append((box, "aspect %.1f>%.1f" % (asp, MAX_ASPECT)))
            continue
        kept.append((box, "ok"))

    # nearest-first, then cap: if we must drop any, drop the FAR ones, which
    # matter least to a 2-6 m lookahead.
    kept.sort(key=lambda kb: -(kb[0][1] + kb[0][3]))
    if len(kept) > MAX_BLOBS:
        rejected.extend((b, "over MAX_BLOBS") for b, _ in kept[MAX_BLOBS:])
        kept = kept[:MAX_BLOBS]

    if debug:
        return kept, rejected
    return [(x + bw / 2.0, y + bh) for (x, y, bw, bh, _), _ in kept]


def _main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    src = argv[1]
    out = argv[2] if len(argv) > 2 else None
    frame = cv2.imread(src)
    if frame is None:
        print("could not read", src)
        return 1
    kept, rejected = detect(frame, debug=True)
    h, w = frame.shape[:2]
    print("frame %s  %dx%d" % (src, w, h))
    print("HSV %s .. %s" % (HSV_LO, HSV_HI))
    print("min blob area %.0f px (%.1e of frame), max %.0f px"
          % (MIN_AREA_FRAC * h * w, MIN_AREA_FRAC, MAX_AREA_FRAC * h * w))
    # Brightness report. A cone lost to darkness is otherwise INVISIBLE in the
    # reject list below -- it fails the colour mask and never becomes a
    # candidate at all -- so surface it explicitly.
    _hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    _vf = value_floor(_hsv)
    _med = float(np.median(_hsv[:, :, 2]))
    print("scene median V=%.0f -> adaptive value floor=%d "
          "(rel %.2f, clamped %d..%d)"
          % (_med, _vf, V_FLOOR_REL, V_FLOOR_MIN, V_FLOOR_MAX))
    _hue_sat = ((_hsv[:, :, 0] >= HSV_LO[0]) & (_hsv[:, :, 0] <= HSV_HI[0])
                & (_hsv[:, :, 1] >= HSV_LO[1]))
    _n_hs = int(_hue_sat.sum())
    _n_dark = int((_hue_sat & (_hsv[:, :, 2] < _vf)).sum())
    print("orange-hue+saturated pixels: %d;  of those TOO DARK to pass: %d (%.0f%%)"
          % (_n_hs, _n_dark, (100.0 * _n_dark / _n_hs) if _n_hs else 0.0))
    if _n_hs and _n_dark > 0.25 * _n_hs:
        print("  ^ a quarter or more of the orange in this frame is being lost to")
        print("    darkness. Lower V_FLOOR_REL (or V_FLOOR_MIN) and re-run.")
    if w <= ANNOT_PREVIEW_W:
        print("NOTE: %dx%d looks like a downscaled /dev/shm PREVIEW. Detection\n"
              "      in production runs on the full 1280x720 frame. Thresholds\n"
              "      are fractional so they transfer, but blob PIXEL areas\n"
              "      printed below are preview-scale." % (w, h))
    print("\nKEPT %d:" % len(kept))
    for (x, y, bw, bh, area), _ in kept:
        print("  box x=%-4d y=%-4d %3dx%-3d area=%-6d -> ground pixel (%.0f, %d)"
              % (x, y, bw, bh, area, x + bw / 2.0, y + bh))
    from collections import Counter
    print("\nREJECTED %d:" % len(rejected))
    for reason, cnt in Counter(r for _, r in rejected).most_common(8):
        print("  %-28s x%d" % (reason, cnt))
    big = sorted((b for b, r in rejected if r.startswith("area>")),
                 key=lambda b: -b[4])[:3]
    for b in big:
        print("  (largest rejected-as-too-big: %dx%d area=%d)" % (b[2], b[3], b[4]))
    if out:
        vis = frame.copy()
        for (x, y, bw, bh, _), _ in kept:
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.circle(vis, (int(x + bw / 2.0), y + bh), 4, (255, 0, 255), -1)
        for (x, y, bw, bh, _), _ in rejected:
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 0, 255), 1)
        cv2.imwrite(out, vis)
        print("\nwrote %s (green=kept, magenta dot=ground point, red=rejected)"
              % out)
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(_main(sys.argv))
