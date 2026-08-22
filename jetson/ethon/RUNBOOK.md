# Ethon Operator Runbook

Team 1360 Electrathon autonomy stack — Jetson Orin NX, 4x Kraken X60 (Phoenix 6 over CANivore), CSI + perc-1 Pi cameras, ROS2 Humble.

All commands run on the Jetson as user `jetson` unless noted. ROS env for manual commands:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## 1. Boot states and mode switching

| Unit | Default | What it does |
|---|---|---|
| `ethon-can.service` | **enabled** | oneshot: brings up `can0` at 1 Mbit (transceiver bus; CANivore USB is separate and needs no ip-link setup) |
| `ethon-capture.service` | **enabled** | data-collection capture (CSI cams + TRT) — CAPTURE mode |
| `ethon-stack.service` | **disabled** | full autonomy stack via `ros2 launch ethon_stack.launch.py` — AUTONOMY mode |

`ethon-stack` and `ethon-capture` **conflict** (both open the CSI cameras); systemd enforces this via `Conflicts=`. Exactly one should be enabled.

**Switch to AUTONOMY mode (car runs):**

```bash
sudo systemctl disable --now ethon-capture
sudo systemctl enable  --now ethon-stack
```

**Switch back to CAPTURE mode (data-collection drives):**

```bash
sudo systemctl disable --now ethon-stack
sudo systemctl enable  --now ethon-capture
```

Check what is running: `systemctl status ethon-can ethon-capture ethon-stack`.
Stack node output: `~/.ros/log/` (launch logs) and `journalctl -u ethon-stack -f`.

---

## 2. First-time setup checklist

Do these once per vehicle build (and re-do after any mechanical change).

1. **Measure and fill `vehicle.yaml`** — every value marked `MEASURE` is a placeholder:
   - `wheelbase_m` (currently 1.60), tape-measure axle to axle.
   - `steer_col_ratio` (currently 12.0 wheel-deg per road-wheel-deg): turn the steering wheel a known angle, measure road-wheel angle, divide.
   - `steer_belt_ratio` (currently 4.0): count pulley teeth, motor pulley : column pulley.
   - Confirm wheel diameter (26 in) and drive gearing (11.46:1).

2. **CANivore install** (fresh Jetson only):
   ```bash
   sudo curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg"
   sudo curl -s --compressed -o /etc/apt/sources.list.d/ctr.list "https://deb.ctr-electronics.com/ctr2024.list"   # use current-year list
   sudo apt update && sudo apt install canivore-usb
   pip3 install --upgrade phoenix6
   ```
   The `canivore-usb` package provides the kernel module and udev rules. Verify after replug: `lsusb | grep -i ctre` and `ip link | grep can`. Bus name in all code is the constant `"canivore"`.

3. **Bench test the Krakens** — wheels OFF the ground:
   ```bash
   python3 /home/jetson/ethon/bench_test_kraken.py
   ```
   Confirms all 4 devices on the bus (drive IDs 0/1/2, steer ID 3), follower sync, direction, and license status (must show Pro device-licensed for TorqueCurrentFOC).

4. **Homography calibration** (per camera, re-do if a camera is moved):
   ```bash
   python3 /home/jetson/ethon/calibrate_homography.py
   ```
   Place markers at known ground positions in base_link (x forward, y left, metres, origin = rear axle center), click the corresponding pixels, and save. birdseye_fusion loads the resulting homography files at startup.

5. **Steering homing / CANcoder** — CANcoder (CAN ID 4) on the steering column is the absolute reference. If present, set its magnet offset with the road wheels physically straight. If absent, the stack falls back to assuming wheels-straight at boot: **center the steering before powering up**, every time, until the CANcoder is installed.

---

## 3. Arming and disarming

The planner boots **disarmed**: it publishes zero `/cmd_vel` until armed, so the stack can be brought up safely with a driver in the seat.

**Arm:**

1. Stack up: `systemctl status ethon-stack` (or start manually: `ros2 launch /home/jetson/ethon/ethon_stack.launch.py`).
2. Health green: `ros2 topic echo /ethon/health --once` — all subsystems must report OK (cameras, CAN, planner, drive).
3. Arm the planner:
   ```bash
   ros2 param set /cone_corridor_planner armed true
   ```

**Disarm / e-stop (any of):**

- Software e-stop (latching):
  ```bash
  ros2 topic pub /ethon/estop std_msgs/Bool "data: true" -1
  ```
- **Physical e-stop** — cuts motor power. Always reachable by the driver.
- Kill the stack: `sudo systemctl stop ethon-stack`, or `kill -INT <pid>` on a manually launched stack — nodes catch SIGINT and neutral the motors.
- Driver override: the driver can always physically overpower the steering (15 A torque cap), and stale commands stop the Phoenix watchdog feed, disabling all motors within ~0.1 s. Software failure = silent motors, never active ones.

---

## 4. Troubleshooting

| Symptom | Check | Fix |
|---|---|---|
| No cones detected | `ros2 topic hz /ethon/cones`; `journalctl -u ethon-stack \| grep -i birdseye`; does the TRT engine exist? | Confirm `/home/jetson/ethon/models/road_v1_best.engine` present; restart stack; if CSI frames missing see "camera missing" row; re-run homography calibration if positions are absurd |
| Motors won't enable | `ros2 topic echo /ethon/health --once`; is the planner armed (`ros2 param get /cone_corridor_planner armed`)? estop latched? | Clear estop (`data: false` then re-arm), feed requires fresh `/cmd_vel` — check planner alive; check Phoenix Tuner X for device faults/licenses |
| CAN bus dead | `lsusb \| grep -i ctre` (CANivore present?); `journalctl -u ethon-stack \| grep -i phoenix`; `systemctl status ethon-can` | Re-seat CANivore USB; `sudo systemctl restart ethon-can`; check 120-ohm termination and connector at the first Kraken; power-cycle motor controllers |
| Camera missing (CSI) | `journalctl -u ethon-stack \| grep -i nvargus`; `systemctl status nvargus-daemon` | `sudo systemctl restart nvargus-daemon`, then restart the stack; make sure ethon-capture/ethon_display are NOT running (camera contention); re-seat ribbon cable if persistent |
| perc-1 offline | `ping 192.168.0.101` (LAN) / `ping 100.86.169.68` (Tailscale); `nc -zv 192.168.0.101 5001` and `5002` | Power-cycle the Pi 5; check Ethernet/switch; stack degrades gracefully without perc-1 (front+right CSI only) — safe to drive, reduced coverage |
| High temps / derating | `ros2 topic echo /ethon/health --once` (motor temps); `tegrastats` (Jetson) | Drive node auto-derates 80 A -> 50 A over 55-70 C — back off throttle and let motors cool; check airflow over Krakens and Jetson heatsink; do not bypass the derate |
| Stack node crash-looping | `journalctl -u ethon-stack -f` (launch respawns nodes every 5 s, unit itself restarts after 10 s) | Read the traceback; fix or `sudo systemctl stop ethon-stack` to halt the loop |

---

## 5. Remote access

- **SSH (Tailscale, works anywhere):** `ssh jetson@100.70.191.45`
- **perc-1 Pi 5:** `192.168.0.101` (LAN) / `100.86.169.68` (Tailscale); MJPEG TCP ports 5001 (OV5647), 5002 (IMX708).
- **Pull captures to the training PC:** `scripts/pull_captures.sh` (run from the training PC; rsyncs the Jetson capture directory).
- All code lives in `/home/jetson/ethon/`. Service files are copies in `/etc/systemd/system/` — after editing a unit there, `sudo systemctl daemon-reload`.
