# Ground-Plane Camera Calibration Runbook

**This is the one thing blocking autonomy.** Every source currently reads `UNCALIBRATED`
on the dashboard's camera panel because `/home/jetson/ethon/calib/` is empty. Detections
still flow to `/ethon/detections_raw` (debug only), but `birdseye_fusion.py` refuses to
publish anything to `/ethon/cones` or `/ethon/obstacles` for a source with no saved
homography (`birdseye_fusion.py:672`, "uncalibrated: debug topic only, never planner") —
so `cone_corridor_planner.py` never sees a cone and the car cannot drive itself, full stop.

This needs a body standing next to the car with a tape measure. It **cannot be done
remotely** — that is the whole reason this doc exists instead of a script that just does it.

Tool: `python3 calibrate_homography.py` (in this directory, also on the Jetson at
`/home/jetson/ethon/calibrate_homography.py`). Runs headless over SSH, no display needed
on the Jetson itself.

---

## 0. Do this once, in order

**2026-08-11: cameras were regrouped by which platform actually supports them.** The
Jetson now carries the two NVIDIA-supported sensors (IMX477, IMX219) and perc-1
carries both Camera Module 3 Wides (imx708). Reason: an imx708 on the Jetson binds
the *mainline* v4l2 driver, so argus has no ISP tuning for it and never subtracts the
black-level pedestal — with the lens fully covered it produced a flat 103/255 grey
instead of black, and the offset scaled with AE gain, so no fixed correction could
undo it. On perc-1 libcamera ships a real `imx708.json` tuning file and it renders
correctly.

**If you calibrated any source before 2026-08-11, re-run it.** Both Jetson sensor-ids
changed meaning (`cam0` is now the IMX219, `cam1` the IMX477 — the reverse of before)
and every camera's pinned sensor mode changed. See §6.

| # | Source | `--camera`/`--tcp` | Needs fisheye intrinsics first? |
|---|--------|---------------------|----------------------------------|
| 1 | `wide` | `--camera 1` | **Yes — HQ IMX477, 184.6° diagonal fisheye.** Do this one first; it's the hardest and most important (main forward camera). |
| 2 | `narrow` | `--camera 0` | No — IMX219, ~62° FOV. Narrow enough that a plain pinhole homography holds well. Go straight to ground-plane solve. |
| 3 | `left`  | `--tcp 5001` | **Recommended** — Camera Module 3 Wide, ~120° diagonal FOV, wide enough that corners will be measurably off without undistortion. Judgment call: if the corridor logic only needs points near image-centre for this camera's mounting, you can skip it and revisit if `--test` shows bad off-centre points. |
| 4 | `right` | `--tcp 5002` | **Recommended** — same module and same judgment call as `left`. |

Do them in this order so you learn the (harder) intrinsics workflow on the camera that
needs it most, then repeat what you know for the others.

**Sensor-ids are not what you'd guess from the connector labels.** `narrow` is
physically on CAM0 and `wide` on CAM1, but argus numbers them by
`tegra-camera-platform` module order, which here gives `narrow` = sensor-id 0 and
`wide` = sensor-id 1. Don't infer sensor-id from the `/dev/videoN` number either —
those have been observed to differ. The authoritative check is the module badge:
`cat /proc/device-tree/tegra-camera-platform/modules/moduleN/badge`
(`RBP194` = IMX219, `RBPCV3` = IMX477).

---

## 1. What you need physically

- A tape measure (a laser distance measure is faster and more accurate for the far markers).
- 4+ flat ground markers per camera you're solving — tape crosses, chalk dots, or anything
  with a sharp, unambiguous centre point. Re-usable; you move the same markers between cameras.
- For `wide` (and `left` if you choose to do intrinsics there): a **printed
  checkerboard**, 9×6 inner corners, flat and rigid (tape/glue to cardboard — a floppy
  printout ruins the corner-finder). `--square` defaults to 0.025 m (25 mm squares); measure
  your actual printout and pass `--square <metres>` if it differs — this only matters for
  scale sanity, not for the pattern-detection step itself.
- A laptop/phone to view snapshot images (to read off pixel coordinates) — GIMP, Preview,
  even MS Paint's cursor-position readout all work.
- Car should be **stationary and disarmed** for the whole process — obviously, since you'll
  be standing in front of it with a tape measure. `allow_unhomed_steering`/arming state don't
  matter for this; the fusion/calibration path is independent of the drive stack.

All coordinates are **vehicle frame**: origin at the **rear-axle centre**, x = metres
**forward**, y = metres **left** (right is negative). Every measurement is relative to that
point, not to the camera.

---

## 2. Marker layout

Use the tool's suggested pattern as a starting point — spread front-left, front-right, near,
and far, never collinear (the tool will refuse a collinear solve outright):

```
                         forward (+x)
                              ^
                              |
                (-1.5, 6) o       o (1.5, 6)      <- far pair, wide spread
                              |
                              |
              (0, 3) side     |
                              |
               o(1.5,3)       |      <- near pair
                              |
    y=left  <------------[REAR AXLE]------------>  y=right (negative)
                         (origin, x=0)
```

(The tool prints this exact suggested pattern — `(3.0, 0.0), (6.0, 0.0), (3.0, 1.5), (6.0, -1.5)`
— every time you run a plain capture. Markers should sit **>= 2 m apart**, covering both
near and far field within the camera's view. More than 4 points is fine and improves the fit
(the tool switches to RANSAC automatically above 4 points, so an occasional bad measurement
won't wreck the whole calibration).

**Forward-facing cameras** (`wide`, `narrow`): markers go in front of the car, spread across
the lane the corridor planner will drive through. (Naming assumes `narrow`'s physical mount
still points forward after moving to perc-1 — confirm this against the actual mounting before
placing markers; if it was re-aimed during the move, treat it like `left`/`right` below instead.)

**`left`** (and `right`): markers go to that side of the car, in whatever ground area the
camera actually covers — check the snapshot first (step 3) to see its field of view before
placing markers; a side-mounted wide camera may see mostly to the side/rear-quarter, not
straight ahead.

---

## 3. Fisheye intrinsics (wide, and left/right if you choose to)

Skip to §4 if the camera doesn't need this (`narrow`, or `left`/`right` if you
decided against it).

```bash
# 12-20 shots, varied angle AND distance, and make sure the checkerboard
# fills the CORNERS of the frame at least a few times -- that's where the
# fisheye distortion actually lives, a bunch of dead-centre shots won't help.
python3 calibrate_homography.py --camera 1 --shot   # repeat 12-20x, moving the board each time

# Then solve K/D from the captured shots:
python3 calibrate_homography.py --camera 1 --calib-intrinsics
```

- Needs **8 minimum**, tool recommends **12-20**. It tells you live which shots it found
  a pattern in.
- Target **RMS reprojection error < 1.5 px** — the tool warns if you're above that
  ("more varied angles, and make sure the board is FLAT").
- If `cv2.fisheye.calibrate` throws, the tool tells you to delete blurry/partial shots and
  retry — a single bad shot (motion blur, board half out of frame) can break the whole solve.
- Saves `calib/cam1_fisheye.npz` (K, D, rms, image_size). This is picked up automatically
  by the next `--solve` on the same source — you don't pass it explicitly.
- **Re-run `--solve` after this** even if you already ran it once — the tool will print a
  reminder ("NOW RE-RUN --solve: H must be refitted in the undistorted space"). A homography
  fit on raw pixels and one fit on undistorted pixels are not interchangeable; mixing them
  silently produces wrong ground positions everywhere except dead-centre.

For `left`/`right`, same commands with `--tcp 5001` / `--tcp 5002` in place of
`--camera 1`.

---

## 4. Ground-plane solve (every camera)

```bash
# 1. Grab a snapshot (also prints these instructions again with the exact command
#    to run once you have pixel reads):
python3 calibrate_homography.py --camera 1

# 2. Copy it somewhere you can view it:
#    scp jetson@<jetson-ip>:/home/jetson/ethon/calib/cam1_snapshot.jpg .
#    Open it, hover/click each marker, read off (u = column from left, v = row from top).

# 3. Solve (order of points doesn't matter, need >= 4):
python3 calibrate_homography.py --camera 1 --solve \
    "640,580,3.0,0.0 652,470,6.0,0.0 410,560,3.0,1.5 885,468,6.0,-1.5"
```

Read the printed table carefully — it shows measured vs. projected (x, y) and the per-point
error in metres:

- **Refuses to save if mean error > 0.25 m.** If it refuses: re-check that u/v weren't
  swapped (u is the column, i.e. left-right; v is the row, i.e. up-down — easy to
  transpose), re-check tape measurements, and confirm every marker actually sat flat on
  the ground (a marker on a curb or slope is off-plane and the homography model doesn't
  know that).
- A point flagged `<< RANSAC OUTLIER` (only shown with 5+ points) means that one
  correspondence disagrees badly with the rest — re-measure or re-read that one pixel
  before trusting the fit.
- On success it writes `calib/cam1_H.npy` (the matrix) and `calib/cam1_H.json` (a sidecar
  recording whether this H expects undistorted input — `birdseye_fusion` reads this at
  startup, so don't hand-edit or copy one camera's `_H.npy` onto another's filename).

Repeat for `narrow` (`--camera 0`), `left` (`--tcp 5001`) and `right` (`--tcp 5002`).

---

## 5. Sanity-check before trusting it

```bash
python3 calibrate_homography.py --camera 1 --test "640,500"
```

Projects a single pixel through the saved H (undistorting first if intrinsics exist for
that source — this exercises the *exact* path fusion uses, not a simplified one). Pick a
pixel you can eyeball in the snapshot (e.g. a point on the ground a known rough distance
ahead) and confirm the printed range/bearing looks plausible. A negative `x` (behind the
rear axle) on a pixel that's clearly in front of the car in the image means the calibration
is wrong — the tool prints this warning itself.

**Then confirm end-to-end**: restart `birdseye_fusion` (this does *not* require restarting
`ethon-stack` as a whole if fusion runs standalone — check `systemctl status`; if it's part
of the stack launch, this is one of the few reasons a stack restart is justified, per the
overnight-session hard constraint against restarting it casually) and watch the dashboard's
camera panel — the source should flip from `UNCALIBRATED` to showing a `cal` badge, and
`/ethon/cones` should start publishing non-empty `PoseArray`s once real cones are in view.
`ros2 topic echo /ethon/cones` is the fastest way to confirm from the CLI.

---

## 6. Known gotchas

- **`--host` override for perc-1 (`left`/`right`)**: `PERC1_HOSTS` in
  `calibrate_homography.py` tries `perception-1.local` first, then `192.168.2.61`, then the
  Tailscale IP `100.107.192.42` — matching `birdseye_fusion.py`. If a perc-1
  capture still fails to connect (DHCP lease drift, wrong network), pass
  `--host <current-ip-or-hostname>` explicitly rather than editing the script.
- **u/v transposition** is the most common cause of a refused solve — u is horizontal
  (column), v is vertical (row). If every point is off by roughly the same wrong amount,
  suspect this first.
- **Fisheye solved but points near the image edge still fail**: the tool caps undistortion
  at `UNDISTORT_MAX_NORM = 5.0` normalised units (~79°). A marker placed at the extreme edge
  of a 184°-diagonal lens's view may legitimately be unrepresentable — move it slightly
  inward rather than fighting the math.
- **A stale `_H.npy` from an old mounting position will NOT auto-invalidate.** If you
  physically move or re-aim a camera, re-run the whole solve for that source — there's no
  drift detection, it'll just silently give wrong ground positions from the old geometry.
  **This also applies when a camera moves to a different sensor-id/port**, which is exactly
  what happened 2026-08-11 (see the top of this doc): the two Jetson sensor-ids swapped
  meaning, so any pre-existing `cam0_H.npy`/`cam1_H.npy` would now be applied to the *other*
  physical camera — a different lens, mount and field of view — and would silently give
  wrong ground positions. Not an issue right now (`calib/` was verified empty), but keep it
  in mind for any *future* camera move.
- **A changed sensor mode invalidates a homography just like a physical move.** Both Jetson
  sources pin `sensor_mode` in `birdseye_fusion.py` to avoid argus defaulting into a
  reduced-FOV mode. If anyone changes those pins, or the boot overlay, the field of view
  shifts under the saved `H` and that camera must be re-calibrated.
- **Vehicle must not move** between snapshot capture and marker measurement (obvious, but
  worth stating: don't let anyone lean on the car or roll it while you're mid-calibration).
- Calibration files are keyed by connector (`cam0`, `cam1`, `cam_tcp5001`, `cam_tcp5002`), NOT
  by the human-readable source name — renaming entries in `birdseye_fusion.py`'s `SOURCES`
  list is safe as long as `sensor_id`/`port` stay pointed at the same physical camera. It is
  moving a camera to a *different* sensor-id/port that invalidates the old file, per above.

---

## 7. When you're done

All four expected sources (`wide`, `left`, `narrow`, `right`) should show `cal` on the
dashboard once calibrated — check the dashboard's camera panel for current alive/disconnected
status rather than trusting a fitted/not-fitted assumption written here, since that changes
as hardware gets connected. At that point cones and obstacles reach the planner and — separately, and not
covered by this doc — the car can be armed. See `PROJECT_HANDOFF.md` for what still needs
review before that's a good idea even once calibration is done (hazard class coverage,
stop-distance margin).
