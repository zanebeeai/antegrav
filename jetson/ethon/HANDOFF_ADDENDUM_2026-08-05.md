# Addendum for PROJECT_HANDOFF.md — 2026-08-05 overnight session

**Not yet merged into `/home/jetson/ethon/PROJECT_HANDOFF.md`.** This session's SSH access
to the Jetson was blocked the entire run by a Tailscale SSH re-auth challenge
("Tailscale SSH requires an additional check" — requires a browser login only Arnav can
complete; see session summary). I don't have the current live `PROJECT_HANDOFF.md` text to
edit safely, so rather than guess at its structure and risk clobbering something, this is a
standalone delta — merge it in by hand or hand this file to a future session with jetson
access and ask it to fold this in.

Everything below is drawn from the project memory system (all dated 2026-08-03/08-04
entries) and cross-checked against the local repo mirror where the relevant file exists.

---

## Camera sources — renamed, now named for where they point

`birdseye_fusion.py`'s `SOURCES` list (confirmed in the local mirror, `birdseye_fusion.py:266-269`):

| Name | Kind | Hardware | FOV | Status |
|---|---|---|---|---|
| `front_wide` | CSI sensor-id 0 (Jetson) | HQ IMX477 | 184.6° diagonal fisheye | streaming ~9-10 fps |
| `front_far` | CSI sensor-id 1 (Jetson) | IMX219 | ~62° | streaming ~9-10 fps |
| `side_left` | TCP :5001 (perc-1) | Camera Module 3 Wide | ~120° | CONNECTED, streaming ~9-10 fps |
| `side_right` | TCP :5002 (perc-1) | — | — | **not fitted**, expected disconnected |

## perc-1 rebuilt — no ROS anymore

perception-1 (Pi 5) was rebuilt on plain Raspberry Pi OS with **no ROS2 stack at all**.
New capture path: `/home/pi/fleet_stream.py` — picamera2 → JPEG → length-prefixed TCP,
wire-format-compatible with what the Jetson already expects (4-byte big-endian size prefix
+ JPEG, same as the old `image_tx`/`fleet-imagetx` relay). Runs as `fleet-camera.service`.
Local mirror copies live under `perc1/` in this repo.

**Raw sensor mode is pinned to 2304x1296.** Left on auto, libcamera picks 1536x864, which
is a *centre crop*, not a downscale — silently loses ~1/3 of the field of view. This is
exactly the kind of thing that would quietly break calibration if someone re-ran
`fleet_stream.py` without the pin.

Also see: perc-1's SSH host key rotated as part of this rebuild (expected — full OS
reinstall generates a new key — not a security incident, just update `known_hosts` after
confirming it's really perc-1).

## Fisheye undistortion is now required for the wide cameras

`front_wide`'s 184.6°-diagonal lens badly breaks the pinhole assumption a plain homography
relies on, off-axis. `calibrate_homography.py` gained `--shot` / `--calib-intrinsics` modes
(fisheye K/D solve from checkerboard captures) and `birdseye_fusion.py` gained
`undistort_points_fisheye` to apply it before projecting through H. See the new
`CALIBRATION.md` in this directory for the full workflow — **this is the single blocker on
autonomy right now**: `calib/` is empty, so every source reads `UNCALIBRATED` and nothing
reaches the planner.

## Cone de-duplication added to fusion

`_merge_duplicates` in `birdseye_fusion.py`, 30 cm radius — prevents the same physical cone
being double-published when two cameras' fields of view overlap.

## Steering hand-back rule (important, safety-relevant)

`ethon_drive.py` now subscribes to the planner's **latched** `/ethon/hmi/armed` topic and
releases the steer motor to `NeutralOut` whenever the car is disarmed, so the driver can
always turn the wheel by hand. Previously the steer motor servoed to centre and physically
fought the driver while disarmed — because the planner keeps publishing zero `/cmd_vel`
while disarmed, `/cmd_vel` freshness alone never told the drive node that autonomy wasn't
actually driving. **Gate steering release on `armed`, never on cmd-velocity freshness** —
this is now a hard rule for any future touch of that code path.

## Dashboard additions

- Camera panel showing all 4 sources with live thumbnails/status.
- Source-status strip surfacing `calibrated` (was previously silent — a source could be
  uncalibrated with no visual indication at all) — shows `UNCALIBRATED` until a homography
  is saved for that source, `cal` badge after.
- Steering visualiser below Telemetry: wheel rotates with the column, rev-LED-style strip,
  and a travel arc that goes amber near lock.
- Predicted-path arc that tracks the wheel without a frame of lag.

## Jetson timezone was wrong

Factory default was `Asia/Shanghai`. Now set to `America/Toronto` with `set-local-rtc 0`
(the Orin has no RTC battery, so it reads 1970 until NTP syncs on boot — this is expected,
not a fault, if you ever see a Jan-1970 timestamp in an early boot log).

## Steering homing / non-FOC path (2026-08-03, larger change — see also `vehicle-roborio`/`project-jetson-handoff` memory for full detail)

Lock-to-lock homing is now real: sweeps RIGHT then LEFT to the mechanical stops under a
tuneable stator-current cap (`steer_home_current_a`, default 8 A, restored to `steer_max_a`
in a `finally`), centres on the midpoint, derives soft limits from the measured range.
**As of the last verified read, no real range had been measured** — the steer motor was not
yet coupled to the steering rack, and bench sweeps were stopped by hand at arbitrary points.
Re-run homing once the rack is physically coupled before trusting any derived soft limit.
Added a non-FOC steering path (`PositionDutyCycle`, gain slot 1) alongside the existing FOC
path (`PositionTorqueCurrentFOC`, slot 0), switched by the same `use_foc` toggle as drive,
capped by `steer_peak_duty` (0.25). **All steering gains are untuned starting points** —
don't assume they're safe defaults without a bench check.

---

**If you're the session picking this up**: try jetson SSH first — if the Tailscale check is
still blocking, that's worth mentioning to Arnav directly since it'll block every future
unattended overnight session the same way until he re-authenticates or the ACL's check
period is adjusted. See the full session summary for the exact error text and what was tried.
