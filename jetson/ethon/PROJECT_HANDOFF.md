# ETHON — Full Project Handoff

**Purpose:** durable record of everything worked on, so the project survives a laptop change.
This file lives on the Jetson at `/home/jetson/ethon/PROJECT_HANDOFF.md`. It supersedes the
older `HANDOFF.md` (which had the wrong team number). Last major update: **2026-07-04**.

---

## 0. CRITICAL — access & credentials (read first if on a new laptop)

- **Jetson Orin NX** — hostname `yahboom`, user `jetson`, **sudo password `yahboom`**.
- Reach it two ways:
  - **Tailscale** (works anywhere): `ssh jetson@100.70.191.45`. Requires Tailscale installed
    on the new laptop and logged into the **same tailnet** (`arnnav0kudale@gmail.com`).
    Tailscale SSH sometimes lags 30–60 s on reconnect / relays through "tor" — just retry.
  - **LAN** (same WiFi as the car): `ssh jetson@192.168.2.77`. Jetson WiFi is on
    `192.168.2.0/24`. This bypasses Tailscale entirely and is more reliable when on-site.
- **Web dashboard:** `http://100.70.191.45/dashboard` (Tailscale) or
  `http://192.168.2.77/dashboard` (LAN). Also `/pit` and `/replay`.
- **To keep access from the new laptop:** install Tailscale + log into the tailnet, OR copy the
  SSH key/authorized access. Without one of those you lose remote access to the car computer.
- Project working dir on the laptop was `C:\Users\anupk\Desktop\ethon`. Source of truth for
  code is the Jetson at `/home/jetson/ethon/`.

---

## 1. What Ethon is

- A **WOSS** (White Oaks Secondary School, Oakville ON) team converting a pedal Electric
  Vehicle into a self-driving car. This is a **Waterloo EV Challenge** team (high-school
  battery-EV endurance race), **team number 843** — NOT a FIRST/FRC team. (Team 1360 is their
  *separate* FRC robotics team — relevant to licensing, see §7.)
- President / main contact: **Arnav Kudale** (incoming Grade 12, summer 2026).
- 2026 results: **3rd in the feature race**, **3rd for the Multimatic Engineering Design Award**.
- Carries a human driver who can always override.

## 2. Hardware / architecture

- **Jetson Orin NX 8 GB** — perception + control (RoboRIO was removed 2026-06-11).
- **4 cameras** (2 CSI on Jetson + 2 on a Pi 5 "perc-1"), unified YOLO TRT model `ethon_v1`.
- Pipeline: cone-corridor planner → pure pursuit → **4× Kraken X60** motors via **Phoenix 6**.
- **Steering-wheel hub = Raspberry Pi Pico (CircuitPython)** on the wheel: owns the Nextion
  display UART, the NeoPixel LED strip, and the wheel buttons/encoders. Talks to the Jetson over
  ONE USB cable (framed link). See §5.
- **GPS**: Radiolink M10N / u-blox M10 @ 38400 on `/dev/ttyTHS1` → NavSatFix on `/gps/fix`.
- **CAN**: the 4 Krakens are on the **Jetson's native CAN controller** (`can0`, `mttcan`),
  NOT a CANivore (there is no CANivore hardware). Through an SN65HVD230 transceiver. See §6.

## 3. Systemd services (all `User=jetson`, enabled at boot unless noted)

| Service | What it runs | Notes |
|---|---|---|
| `ethon-stack` | Autonomy launch: birdseye_fusion, cone_corridor_planner, **ethon_drive**, health_monitor | The drive node lives IN here. `ethon-drive.service` is legacy/disabled. |
| `ethon-capture` | Capture mode | **Disabled** — autonomy is the permanent default. |
| `ethon-gps` | `gps_driver.py` → `/gps/fix` | |
| `ethon-lap` | `lap_timer.py` → `/ethon/lap` | GPS lap timer |
| `ethon-wheel` | `wheel_bridge.py` (Nextion + LEDs + buttons via Pico) | Supersedes old `ethon-hmi` |
| `ethon-can` | brings up `can0` classic 1 Mbps | see §6 — has a drop-in override |
| `ethon-dashboard` | `web_dashboard.py` on port 80 | see §4 |
| `ethon-race` | `race_services.py` (3 nodes: strategist, corridor, session logger) | see §4 |

Restart pattern: `echo yahboom | sudo -S systemctl restart <svc>`.
Clearing E-STOP restarts `ethon-stack` (the drive latches estop until its process restarts).

## 4. Web dashboard (`web_dashboard.py`, ethon-dashboard.service)

Pure-Python-stdlib HTTP server + ROS2 node. Subscribes to **every** topic generically
(`rosidl_runtime_py.message_to_ordereddict`), serves a single-page UI. Reachable at
`/dashboard`, `/pit`, `/replay`.

Features built this project:
- **Live topics** (auto-decoded JSON), **live parameter editor** (List/Get/Set on
  PARAM_NODES = planner, lap_timer, ethon_drive, gps_driver, birdseye_fusion, health_monitor,
  **race_strategist, corridor_warning**), **action buttons** (ARM/DISARM/E-STOP/CLEAR E-STOP/
  MARK/MODE/START RACE).
- **Track map** (GPS/world, offline canvas, no tiles) — track + current pos + start/finish +
  geofence circle. Lap timer publishes `line_lat`/`line_lon`/`geofence_m` for it.
- **Cones & plan bird's-eye** (robot frame) — `/ethon/cones` dots + `/ethon/path` line + car.
- **Telemetry charts** (~5 min history, 4 Hz): speed / energy Wh / Wh-per-km / max motor temp.
- **Log pane** — `/rosout` ring, colour-coded, level filter.
- **Battery** — top status card shows **measured** pack voltage from a Kraken (green ≥11.5,
  amber 10.5–11.5, red <10.5). Distinct from the strategist's Wh-budget "estimate".
- **Self-test** (SELF-TEST button, `/api/selftest`) — topic freshness, GPS fix, CAN motors,
  estop, config_hold, Pico device present, wheel_bridge alive, disk space → READY/NOT READY.
- **Motor bench test** panel — DUTY/FOC toggle + duty slider + fwd/rev + START/STOP, live
  per-motor rps/current. Posts `/ethon/drive_test` (Float64 duty), watchdog-safe (§7).
- **/pit** page — big tiles: pace verdict, battery %, burn rate, Wh/lap, regen %, temps, alerts.
- **/replay** page — pick a logged session, see speed/energy charts, GPS track, per-lap Wh table.

**ethon-race.service** (`race_services.py`), 3 nodes:
- `race_strategist` — energy pacing. Battery = Interstate **MTX-35 (12 V 55 Ah AGM ≈ 660 Wh
  nominal)**; params `battery_usable_wh=480`, `race_minutes=70` (now dashboard-tunable).
  `/ethon/race/start` (START RACE button) starts the clock. Publishes `/ethon/strategy` 1 Hz:
  wh used/remaining, burn rate vs budget, pace verdict (±8 % band), Wh/lap. Survives drive-node
  energy-integrator resets via an offset accumulator. (Divide-by-zero guarded 2026-07-04.)
- `corridor_warning` — lateral deviation of the planned midline from the car; states
  no_path/ok/warn/off at 0.5/0.9 m; `/ethon/corridor` → wheel WARN zone + amber LED + dashboard.
- `session_logger` — CSV telemetry at 2 Hz to `/home/jetson/ethon/logs/`, opens on
  armed/rolling, rotates after 5 min idle. Read back on the /replay page.

**Dashboard gotcha:** binds port 80 via `AmbientCapabilities=CAP_NET_BIND_SERVICE`. Do **NOT**
also set `CapabilityBoundingSet=...` — a tight bounding set silently breaks the setuid `sudo`
that CLEAR E-STOP needs. No auth on the dashboard — keep it on a trusted network (it can arm/estop).

## 5. Steering-wheel hub (Pico) + Nextion + lap timer

- `pico/code.py` (CircuitPython) owns: Nextion UART (GP0/GP1, 9600), NeoPixel strip, wheel
  buttons + 3 rotary encoders. Framed USB link to `wheel_bridge.py` on the Jetson
  (`/dev/ethon-wheel`, udev `99-ethon-usb.rules`).
- **Buttons (BUTTON_MAP):** GP5=arm, GP17=disarm, GP6=estop, GP16=mark, GP13=mode.
- **Encoder 1** (GP12/GP10, sw quadrature) → planner `target_speed_ms` ±0.5 m/s/detent.
- **Encoder 3** (GP4/GP2) → drive `regen_strength` ±0.1 (floor 0.2 — regen is the only brake).
- **Encoder 2** (GP18/GP20) → DEBUG `/cmd_vel` speed setpoint for bench testing (bypasses planner).
- **NeoPixel data pin MOVED GP2 → GP22** (GP2 freed for encoder 3). Rewire the LED data line to GP22.
- **RP2040 `rotaryio` needs SEQUENTIAL GPIO** (PIO); ours aren't adjacent → SOFTWARE quadrature
  decode (digitalio + Gray-code table) in code.py.
- **Clear E-STOP gesture:** hold **ARM + DISARM together 3 s** → Pico emits one-shot
  `estop_clear` token → wheel_bridge publishes `/ethon/estop false` + runs `ethon_clear_estop.sh`
  (restarts ethon-stack). Dashboard CLEAR E-STOP does the same.
- **Deploy Pico:** write `pico/code.py`, copy to CIRCUITPY mass-storage (device letter MOVES —
  it's been `/dev/sdb1` and `/dev/sda1`; check `lsblk | grep -i circuit`), `sync`, unmount, then
  force a soft reload over `/dev/ethon-wheel-console` (Ctrl-C ×2, Ctrl-D). Auto-reload lags minutes.
- **Nextion:** genuine NX4832F035, 480×320, 9600 baud, runtime-drawn (no .tft app running now).
  Has NO built-in fonts — needs font 0 in the .tft or xstr text is invisible. Orientation set in
  the .tft only.
- **DO NOT baud-scan `/dev/ttyTHS1`** at high bauds — it wedges the Tegra UART until a reboot.
- **Lap timer** (`lap_timer.py`): proximity + hysteresis. Start/finish auto-set at first GPS fix
  (or MARK button). Must leave >2× geofence (arm), return within geofence, ≥ min_lap_s to score.
  Publishes `/ethon/lap` at 10 Hz. Params: geofence_radius_m=20, min_lap_s=15, arm_factor=2,
  auto_set_on_first_fix=true. `fix` has a 3 s debounce so it doesn't flicker on brief dropouts.

## 6. CAN bus — the big fix (see also §7)

- Motors are on the **Jetson native `can0` (mttcan)**, through an **SN65HVD230** transceiver.
  Confirmed via candump/cansend: CTRE 29-bit IDs, manufacturer 4.
- **The bug:** `ethon-can` brought the bus up as CAN-FD 2 Mbps (`dbitrate 2000000 fd on`).
  The SN65HVD230 is rated ~1 Mbps and **cannot drive the 2 Mbit FD data phase** → TX bus-off →
  every motor read `faults:["unavailable"]`. FRC-mode Krakens speak **classic CAN 1 Mbps**.
- **The fix (APPLIED):** systemd drop-in `/etc/systemd/system/ethon-can.service.d/override.conf`
  brings `can0` up classic: `ip link set can0 up type can bitrate 1000000 restart-ms 100` (no fd).
  After this, Phoenix 6 enumerates the Krakens over classic SocketCAN — real temps, `faults:[]`.
- **`can_bus` hardcoded to `can0`** in vehicle.yaml (was `canivore` with a fallback that RACED
  and got stuck reporting motors unavailable). No CANivore exists.
- **Device IDs (renumbered by user 2026-07-04):** **1 = drive master, 2 & 3 = followers,
  4 = steer**. CANcoder is also id 4 — that's FINE, Phoenix allows the same numeric ID across
  different device *types* (Kraken vs CANcoder).
- **mttcan gotcha:** control-mode flags are sticky. After a listen-only/fd session, pass
  `fd off listen-only off` explicitly or the interface stays LISTEN-ONLY (cansend silently no-ops).

## 7. Drivetrain: why the motors wouldn't spin, and the fix

Two stacked, independent gates — both understood:
1. **FRC Lock** (fixed by user in Tuner X → factory-default each device). The Krakens had been
   on a roboRIO (removed 2026-06-11); CTRE FRC-Lock then refuses to enable outside a roboRIO,
   silently (`device_enable` stays DISABLED on ANY control mode, reads still work). Factory
   default clears it. Confirmed: a Kraken then enabled + spun.
2. **Phoenix Pro license for FOC** (worked around in code). The drive commanded
   `TorqueCurrentFOC` — **FOC is a Pro-licensed per-device feature**; on non-FRC SocketCAN the
   master raised `fault_unlicensed_feature_in_use` and produced zero output. Direct read:
   `is_pro_licensed=False` even though Tuner X showed a Season-Pass `LIC-...-FRC-01360` badge
   (team 1360). A power cycle didn't change it — **Season Pass appears tied to real FRC-robot
   use, not portable to the Jetson**. (Spoofing a roboRIO/team number to unlock it was asked
   for and **declined** — that's licensing circumvention. Path forward: CTRE support about a
   standalone Device License, or stay on the non-FOC path below.)

**The fix — `use_foc` param + bench test (in `ethon_drive.py`, deployed + CONFIRMED motion):**
- New param **`use_foc`** (default **false** in vehicle.yaml). false = `DutyCycleOut` (open-loop,
  NO license); true = `TorqueCurrentFOC` (needs Pro). **Toggle live from the dashboard** bench
  panel — one click, reversible, no redeploy. The single-pedal torque map applies as
  `duty = amps / thermal_limit` when FOC is off. Added a **stator current limit** so duty mode is
  stall-safe. With use_foc=false the master's unlicensed fault clears.
- **Bench test:** `/ethon/drive_test` (Float64 duty −1..1). Fresh+nonzero OVERRIDES `/cmd_vel`,
  applies DutyCycleOut directly (bypasses torque map + steering), always non-FOC. Watchdog 0.5 s
  (stop re-posting → coast). Capped to `test_max_duty` (0.30). **CONFIRMED 2026-07-04: master +
  follower spun ~10 rps at 8 % duty, follower mirrored, clean stop — first commanded drivetrain
  motion of the project.** Respects estop + config_hold. **Wheels OFF the ground for any test.**
- CAVEAT: DutyCycleOut is open-loop — speed sags under load, coarse vs FOC. Fine for bench /
  getting moving; FOC (if ever licensed) is tighter for racing.

## 8. Steering (hardware NOT wired yet)

- Uses a **CANcoder on the steering column/wheel** for absolute position; steer Kraken = id 4.
- **Homing reworked to lock-to-lock** (user's preferred method): sweep steering fully right, fully
  left with open-loop duty, centre = midpoint of the two extremes, derive soft limits from the
  measured range. Selectable via `steer_home_method` in vehicle.yaml (`cancoder` | `lock_to_lock`).
  **lock_to_lock is UNTESTED — no steer hardware on the bus yet.** Tune the `STEER_HOME_*`
  constants against real hardware.
- **WARNING:** operational steering still uses `PositionTorqueCurrentFOC` = a Pro-licensed FOC
  feature → once steering is wired it will hit the **SAME licensing wall** as the drive did. Will
  need a use_foc-style non-FOC path (PositionVoltage / PositionDutyCycle + PID) or a license.
  Homing itself uses non-FOC duty (no license).
- Still-to-MEASURE steering values: `steer_col_ratio`, `steer_belt_ratio`, `steer_limit_rot`,
  `cancoder_offset_rot` (+ `cancoder_invert`). CANcoder absolute range is **±0.5 rotation** — the
  column must boot within half a turn of centre for cancoder homing to be unambiguous.

## 9. Arm/disarm state fix (2026-07-04)

- The ARMED/DISARMED indicator was reading drive_status **`enabled`**, which is only `/cmd_vel`
  freshness and **stays true while disarmed** (planner sends zero cmd_vel) — so DISARM never showed.
- **Fixed:** `cone_corridor_planner` now publishes a **latched `/ethon/hmi/armed`** (Bool,
  TRANSIENT_LOCAL) on every arm change + at boot — the single source of truth. wheel_bridge +
  dashboard read that instead of `enabled`.
- GOTCHA: a latched topic needs a **TRANSIENT_LOCAL** reader to get the last value on connect;
  a BEST_EFFORT/VOLATILE reader does not. Handled in the dashboard `_qos_for` (same fix that
  makes the latched estop read correctly).

## 10. vehicle.yaml — key params (source of truth: `/home/jetson/ethon/vehicle.yaml`)

- `can_bus: can0` (native, not canivore)
- `drive_master_id: 1`, `drive_follower_ids: [2, 3]`, `steer_can_id: 4`, `cancoder_can_id: 4`
- `wheel_dia_m: 0.6604` (26"), `gear_ratio: 11.46`, `max_speed_ms: 8.0`
- **`wheelbase_m: 1.524`** (measured — 5 ft), track width **1.219 m** (48 in; not a drive param,
  feeds the planner's `vehicle_width_m`)
- **`geometry_measured: true`** (set for bench testing — this is the master "car may move" gate)
- **`use_foc: false`**, `test_max_duty: 0.30`
- Steering ratios still MEASURE placeholders (see §8)

## 11. Jetson llama.cpp (local LLM)

- Built from source **with CUDA** (`-DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=87`, `-j4` to not
  OOM). Repo `/home/jetson/llama.cpp`, binaries symlinked to `/usr/local/bin/{llama-cli,
  llama-server,llama-mtmd-cli,llama-gguf-split}`. Models in `/home/jetson/models/`.
- Verified GPU inference (Qwen3-0.6B, ~73 tok/s gen). Run:
  `llama-server -m ~/models/X.gguf -ngl 99 -c 2048 --host 0.0.0.0 --port 8080`.
- **Rules for the 8 GB shared RAM:** always pass `-c 2048` (default context OOMs). 3–4B Q4 (~2.5 GB)
  coexists with the car stack; 7–8B needs `sudo systemctl stop ethon-stack ethon-dashboard
  ethon-race` first; 13B+ won't fit. Declined to set up an "abliterated" (guardrail-stripped) model.

## 12. Non-technical work this project

- **WOSS Open Electrathon Race — event proposal** (a first-annual student EV race hosted at WOSS
  North Campus). Final PDF: `C:\Users\anupk\Desktop\ethon\WOSS_Electrathon_Race_Proposal_Nov14_2026.pdf`
  (black/purple theme). Targets **Sat Nov 14, 2026, entirely on school property** (no road
  crossing — that's deferred to a future year; a McCraney St E crossing needs 6–12 mo Town lead
  time + paid police + transit detour, not feasible for 2026). Budget ~$1.2k–$6.2k (school-lent
  gear + school-sanctioned insurance → realistic ~$1.5–2.5k). Three go/no-go gates (Aug 15
  Principal, Aug 31 venue+insurance, Oct 10 ≥6 teams) else slide to spring 2027. Insurance plan:
  run it **school-sanctioned** (like the Nov 2025 WOSS V5RC tournament, where the school handled
  insurance) rather than as a third-party rental.
- **Emails drafted:** to the Principal (Mr. Graham — may not be principal next year; asked for
  written support + school-sanctioned classification + a continuity owner); a gracious decline
  reply to "Orbit" (team 1360) declining shop use; Polymaker sponsorship reply.
- **Mechanical design advice given** (no CAD produced by me): steering rack clamp (split clamp,
  print orientation, axial knurl in the bore, PPS-CF vs PAHT-CF), knuckle/upright (aluminium
  7075 not titanium; it carries bearing + ball-joint + steering-arm + caliper loads), shock mount
  clevis (print pin-axis vertical), tie-rod extension (**buy a metal M10×1.25 coupler — do NOT
  print a threaded tie-rod link**), brakes (front = emergency backup only; bike 6-bolt hub →
  Shimano SLX/mineral-oil, servo-actuated via the Pico, keep the trigger path off the main
  stack), suspension shock mount motion-ratio, and part material choice (PAHT-CF for
  impact/cyclic parts, PPS-CF for sustained-clamp/heat parts). Grant/sponsor framing: pitch as
  the EV racing team (omit self-driving), ask for money/materials/mentorship (not machining time).

## 13. Raspberry Pi — separate display-only system (IN PROGRESS, not finished)

- User is setting up a **second, fully separate** Pi for the **old / other car** — it should run
  ONLY the **display half**: steering-wheel Nextion display, lap timer, and web dashboard.
  **Remove all autonomy / camera / drive / CAN code.**
- Creds: user `pi`, password `389179`, or the Jetson SSH key. Prefer key/Tailscale.
- Status when this handoff was written: was **discovering the Pi on the LAN** (192.168.2.0/24) —
  it wasn't in ARP/Tailscale yet (freshly powered / possibly still booting or on Ethernet). Next
  step was to probe port 22 + SSH banners to find it, then assess whether it has ROS2 (the wheel
  bridge / lap timer / dashboard are ROS2 nodes — biggest unknown is whether ROS2 needs installing).
- This is a NEW clean target — **do not touch the Jetson config for it.**

## 14. Open items / next steps

- [ ] **Steering:** wire the steer Kraken (id 4) + CANcoder (id 4), measure the steering ratios,
      test lock-to-lock homing, and add a non-FOC steering control path (licensing wall).
- [ ] **`drive_inverted` check** — now that the drive spins, verify +cmd = forward rotation (bench).
- [ ] **Phoenix Pro licensing** — talk to CTRE about a standalone Device License usable off-FRC,
      or keep running non-FOC (use_foc=false).
- [ ] **Pico reflash** whenever the wheel is reconnected (encoders 2/3 + LED-on-GP22 + estop_clear
      gesture) — device letter for CIRCUITPY moves, check `lsblk`.
- [ ] **Race proposal** — send the Principal email; pull the V5RC paperwork for the insurance path.
- [ ] **Raspberry Pi display system** — finish §13.
- [ ] Physically move the LED strip data wire to **GP22**.

## 15. Memory files (won't transfer with the laptop)

Structured notes lived at `C:\Users\anupk\.claude\projects\C--Users-anupk-Desktop-ethon\memory\`
on the old laptop: ethon-project, web-dashboard, nextion-hmi-lap-timer, canbus-native-mttcan,
kraken-drive-control, jetson-llama, grant-fundraising-goal, sponsorship-pitch-framing. They're the
distilled source for this file. This handoff consolidates them; if they don't sync to the new
laptop, this document is the record.
