# Proposal: cone_corridor_planner hazard-reaction gaps

**Status: proposal only, nothing in this doc has been applied.** These are behaviour
changes to the autonomy stack and need Arnav's sign-off before any of them touch
`cone_corridor_planner.py`. Written 2026-08-05 from a static read of the planner
(no live jetson access this session — see the session summary for why); the numbers
below are grep-verified against the current `jetson/cone_corridor_planner.py` in this repo.

## Context: what the planner already does well

Worth stating up front so a fix doesn't accidentally regress these — all present and
correct as of this read:

- Refuses to plan if the two cone walls cross, or if the remaining corridor gap is
  narrower than `vehicle_width_m`.
- Coast-stops after `cone_timeout_s` (1.0 s) of no fresh cone data.
- Requires `arm_vision_time_s` (2.0 s) of *continuous* cone vision before allowing motion —
  not just "cones present right now."
- Obstacle-feed staleness **fails closed**: `OBSTACLE_FRESH_S` (1.0 s) — a dead detector
  stops the car, it doesn't silently disable hazard reaction.
- Hazard stop has release hysteresis (`hazard_release_m`, 1.0 m, applied to both the stop
  range and the lateral bound) so a person standing at exactly the stop boundary can't
  chatter the command at 20 Hz.
- Hazards only count when laterally inside `corridor_half_width_m + hazard_lateral_margin_m`
  — a spectator standing beside the course doesn't false-stop the car.

## Gap 1: only 3 of the model's classes cause any reaction

`HAZARD_CLASS_IDS = {2, 7, 9}` (`cone_corridor_planner.py:66`) — person, cyclist, animal.
Everything else the model detects — **car, truck, barrier, pothole, construction, debris**
— is published on `/ethon/obstacles` (confirmed non-hazard classes still reach that topic;
`_obstacles` is populated from all detections, `HAZARD_CLASS_IDS` is only checked inside
`_plan_speed`, `cone_corridor_planner.py:332`) and then **ignored** by the speed logic. The
planner steers around cones, but nothing bends the path or slows the car for an obstacle
sitting inside the corridor. As written, a traffic cone-marked lane with a fallen barrier
or a parked car in it gets driven into at cruise speed.

Two fixes, increasing in effort:

- **Cheap, fail-safe-direction fix**: widen `HAZARD_CLASS_IDS` to include the obviously
  solid/immovable classes (car, truck, barrier, construction — debris and pothole are
  judgment calls depending on size/severity the model can't currently express). This reuses
  the *existing* stop/creep logic unchanged — same distances, same hysteresis, same lateral
  gating. Low risk, doesn't touch any of the geometry code, and is honestly closer to "stop
  pretending these classes don't exist" than a new feature.
- **Real fix**: obstacles should carve exclusion zones out of the corridor geometry, and the
  planner should refuse to pass if the remaining gap (corridor minus exclusion zone) is
  narrower than `vehicle_width_m` — i.e. treat a large obstacle the same way it already
  treats a too-narrow gap between cone walls. This is a real change to `_build_midline`'s
  geometry, not just the speed gate, so it's more work and needs its own design pass before
  writing code. Flagging as a separate, larger follow-up rather than bundling it here.

Recommendation: do the cheap widening now (it's a one-line set change with no new failure
modes beyond what already exists for person/cyclist/animal), track the corridor-bending
version as a real design task.

## Gap 2: stop-range margin is unmeasured and looks thin

- `hazard_stop_range_m` = 3.0 m, `hazard_slow_range_m` = 8.0 m
  (`cone_corridor_planner.py:97-98`).
- `target_speed_ms` default (cruise) = 5.0 m/s; `PARAM_CAPS["target_speed_ms"]` = 8.0 m/s
  is the hard ceiling even for a runtime `ros2 param set` (`cone_corridor_planner.py:60,86`).
- **Regen is the only brake the autonomy stack controls** — front friction brakes are
  manual/emergency only (per project memory; not re-verified this session since it's a
  hardware/`ethon_drive.py` fact, not something this file encodes).

The gate between "start creeping" (8 m) and "must be stopped" (3 m) gives 5 m of
distance in which to shed speed from cruise via regen alone before crossing into the
stop band — and if a hazard isn't detected until it's already inside the 8 m slow range
(e.g. steps out from behind a parked obstacle at 6 m), there's correspondingly less. At
the 8.0 m/s hard cap that's well under a second of margin even under generous assumptions.

**This isn't a code bug** — the logic (slow, then stop, with hysteresis) is the right
shape. The actual gap is that **nobody has measured this vehicle's regen deceleration
curve**, so there's no evidence 3 m / 8 m are the right numbers rather than guesses that
happen to look reasonable. Recommend, before ever arming near people:

1. A measured braking test (known speed, regen-only, record actual stopping distance) to
   replace the guess with data.
2. Re-derive `hazard_stop_range_m` / `hazard_slow_range_m` from that measurement with a
   safety factor, rather than tuning them by feel.
3. Consider whether `target_speed_ms`'s default of 5.0 m/s (not the 8.0 m/s cap) should be
   the ceiling for any near-people operation regardless of what the corridor allows —
   i.e. a separate, lower speed cap specifically for early testing.

## Gap 3 (latent, lower priority): `geometry_measured` is a single flag gating two unrelated things

Per project memory (2026-07-27 read, `vehicle.yaml`): `geometry_measured: true` was set for
drive-Kraken bench testing, but the *steering* measurement keys (`steer_col_ratio`,
`steer_belt_ratio`, `steer_limit_rot`, `cancoder_offset_rot`) were still placeholders at
that time. `geometry_measured` is described as the master "car may move" gate. This is
currently harmless because `allow_unhomed_steering: false` keeps steering disabled
independently — but it means the flag's name no longer matches what it actually guards
once steering work resumes, and it's easy to flip `geometry_measured` for a legitimate
drive-only reason and forget it also silently green-lights steering-adjacent logic that
assumes real geometry. **Not re-verified against the current `vehicle.yaml` this session**
(the 2026-08-03 steering rewrite likely touched this file) — worth a fresh check before
acting on this one specifically; flagging so it doesn't get lost, not asserting it's still
true today.

## What I'm explicitly NOT proposing

No specific line-edit is included here on purpose. Widening `HAZARD_CLASS_IDS` is a one-set
change simple enough to write in five minutes, but "which classes" and "what creep speed is
appropriate near a parked car vs. a person" are judgment calls about acceptable behaviour
near real people and objects — Arnav's call, not mine to make unattended.
