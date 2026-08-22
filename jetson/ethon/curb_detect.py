#!/usr/bin/env python3
"""Classical-CV road/curb edge detection -- v1, UNTUNED against real curbs.

Why classical CV and not a model: ethon_v1 has no curb/road-edge class (its
14 classes are cone, car, person, pothole, traffic_sign, traffic_light,
barrier, cyclist, motorcycle, animal, debris, construction, emergency,
truck), and there is no ready-made curb dataset the way FSOCO was for cones.
Retraining for this would mean sourcing and labelling new data from scratch.
This sidesteps that: it looks for a boundary directly in the image, no
recognition needed.

Arnav's course is a mix of raised curb, grass-to-pavement colour change, and
"varies" -- a single cue would only catch one of those, so this combines two,
matched with logical OR (missing a real boundary is worse here than a false
edge; the planner-side consumer is expected to sanity-check width/continuity
before this ever reaches motion control):

  * EDGE cue (Canny) -- catches strong-contrast boundaries: a raised curb's
    shadow/height line, a painted stripe. Blind to a gradual colour-only
    transition.
  * COLOR cue -- samples a small patch directly in front of the vehicle each
    frame (assumed drivable: the car is on the road/sidewalk right now) and
    flags pixels whose colour differs from it beyond a threshold. Catches
    colour transitions (grass fading into pavement) the edge cue misses.
    Sampling fresh every frame rather than a hardcoded colour is deliberate
    -- pavement colour varies by location and lighting, but "what's directly
    under the car right now" is always a valid road sample.

ALL thresholds below are STARTING POINTS. There is no real curb/sidewalk
footage to tune against yet -- this has only been smoke-tested against a
garage floor (no real curb present), which proves the CODE runs and produces
plausibly-shaped output, not that the THRESHOLDS are right. Expect to retune
CANNY_LOW/HIGH, COLOR_DIFF_THRESH and MIN_ROAD_FRAC against real footage
before trusting this anywhere near the corridor fallback.
"""

import cv2
import numpy as np

# ---- tunable; all need real-world retuning, see module docstring ---------
CANNY_LOW = 50
CANNY_HIGH = 150
COLOR_DIFF_THRESH = 40.0        # BGR L2 distance from the road-reference patch
REF_PATCH_Y_FRAC = (0.90, 1.0)  # bottom strip of frame = "known road"
REF_PATCH_X_FRAC = (0.40, 0.60)  # centred, narrow -- avoid off-road pixels
                                 # even when the car isn't perfectly centred
N_SCAN_ROWS = 12                # image rows searched per side per frame
ROW_Y_FRAC = (0.55, 0.98)       # only the ground-ish part of frame; above
                                 # this, sky/buildings/horizon dominate and
                                 # are not part of this problem
MIN_ROAD_FRAC = 0.15            # a row where less than this fraction of
                                 # near-centre pixels look like "road" is too
                                 # ambiguous to trust -- skip it, don't guess
CENTRE_HALF_WIDTH_FRAC = 0.125  # how wide a strip around image-centre counts
                                 # as "near centre" for the row sanity check


def _road_reference_color(frame):
    """Mean BGR of the patch directly in front of the vehicle, or None."""
    h, w = frame.shape[:2]
    y0, y1 = int(h * REF_PATCH_Y_FRAC[0]), int(h * REF_PATCH_Y_FRAC[1])
    x0, x1 = int(w * REF_PATCH_X_FRAC[0]), int(w * REF_PATCH_X_FRAC[1])
    patch = frame[y0:y1, x0:x1]
    if patch.size == 0:
        return None
    return patch.reshape(-1, 3).mean(axis=0)


def detect_edges(frame):
    """Find candidate LEFT and RIGHT road-boundary pixels in one frame.

    Returns (left_pts, right_pts): each a list of (u, v) pixel coordinates
    ordered near-to-far (bottom of frame to top). Either list may be empty
    if no confident boundary was found on that side in this frame.

    Method: for each of N_SCAN_ROWS rows spanning ROW_Y_FRAC, walk outward
    from the row's horizontal centre in both directions and report the first
    pixel flagged by EITHER cue. Walking outward from a known-road centre
    (rather than scanning the whole row for any strong pixel) means the
    first hit is actually a road-to-not-road transition, not just the first
    high-contrast thing in the row -- a crack, a leaf, a shadow.
    """
    h, w = frame.shape[:2]
    ref = _road_reference_color(frame)
    if ref is None:
        return [], []

    y0, y1 = int(h * ROW_Y_FRAC[0]), int(h * ROW_Y_FRAC[1])
    if y1 <= y0:
        return [], []
    # Crop to the sampled band BEFORE running Canny/color-distance, not
    # after -- ROW_Y_FRAC only ever looks at ~43% of the frame (the ground
    # in front of the car; sky/walls/buildings above the horizon are not
    # this problem), so computing full-frame edge/color maps was pure
    # wasted work. Measured cost: this alone roughly doubled fusion's
    # per-source fps once curb detection was added (2026-08-13).
    band = frame[y0:y1]
    gray = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, CANNY_LOW, CANNY_HIGH)
    color_dist = np.linalg.norm(
        band.astype(np.float32) - ref.astype(np.float32), axis=2)
    color_mask = color_dist > COLOR_DIFF_THRESH

    left_pts, right_pts = [], []
    # Row indices into `band` (0 .. y1-y0-1); add y0 back when emitting
    # points so callers still get full-frame pixel coordinates.
    rows = np.linspace(band.shape[0] - 1, 0, N_SCAN_ROWS, dtype=int)
    cx = w // 2
    half = max(1, int(w * CENTRE_HALF_WIDTH_FRAC))

    for v in rows:
        row_combined = (edges[v] > 0) | color_mask[v]
        row_color = color_mask[v]
        near = slice(max(0, cx - half), min(w, cx + half))
        # Sanity check: is there enough "looks like road" near centre to
        # trust this row's centre as a valid walk-out start point? A row
        # that's already ambiguous at centre (deep shadow, glare, the car's
        # own bumper in frame) is skipped rather than trusted.
        if (~row_color[near]).mean() < MIN_ROAD_FRAC:
            continue

        # Walk outward from centre, vectorised: argmax on a bool array finds
        # the first True. Original code did this with a Python for-loop over
        # up to 640 pixels x 12 rows x 2 directions per source per tick --
        # measurably expensive at 10 Hz across 4 cameras. np.argmax does the
        # same search in C, not the Python interpreter. any() is required
        # first because argmax returns 0 on an all-False array, which is
        # indistinguishable from "found at index 0" otherwise.
        right_seg = row_combined[cx:]
        if right_seg.any():
            u = cx + int(np.argmax(right_seg))
            right_pts.append((int(u), int(v) + y0))       # back to full-frame v

        left_seg = row_combined[:cx + 1][::-1]  # index 0 == cx, walking outward
        if left_seg.any():
            u = cx - int(np.argmax(left_seg))
            left_pts.append((int(u), int(v) + y0))        # back to full-frame v

    return left_pts, right_pts
