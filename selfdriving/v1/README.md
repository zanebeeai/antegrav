# Self-driving v1 data capture operator guide

This guide covers starting, stopping, monitoring, and debugging the v1 recorder
on the Ethon Jetson. The design and dataset contract are in `DATACAPTURE.md`.

## Connect from this PC

Open a terminal on this PC and connect with:

```bash
ssh jetson@192.168.2.162
```

The remaining commands in this guide run in that SSH session. The deployed
firmware is under `/home/jetson/ethon`.

## Current deployment state

The capture service and wheel integration were deployed on 2026-08-22. The
service is intentionally inactive and is not enabled at boot. The GP19 wheel
button firmware is installed; the former Pico `code.py` is retained on the Pico
as `/code_pre_v1_20260822.py` and a second backup is on the Jetson.

Do not start the first real capture yet. Preflight currently blocks on the
operator-entered run metadata, crop confirmation, and fixed exposure/gain
values described below. This is expected and prevents an undocumented dataset.

## Before the first real run

The recorder is deliberately not enabled at boot. Before collecting data, edit
`/home/jetson/ethon/v1_capture.json` and replace the session placeholders:

- `track_name`, `track_direction`, and `driver_identifier`;
- `weather`, `lighting`, and `nominal_speed_m_s`;
- the final fixed crop, then set `crop_confirmed` to `true`;
- fixed `exposure_range` and `gain_range` for both cameras.

Run the preflight after every configuration change:

```bash
cd /home/jetson/ethon
source /opt/ros/humble/setup.bash
python3 v1_data_capture.py --preflight
```

A successful preflight exits with code 0 and reports no `setup_issues`. The
service refuses to record a real run while required metadata or camera settings
remain unset.

The final 2026-08-22 storage check found 40.17 GB free. At the configured two-camera
bitrate, the guarded requirement is 19.31 GB for 60 minutes or 23.97 GB for the
90-minute service limit. This includes a 10 GB reserve. Recheck free space before
each session because old runs and the legacy `capture_data` directory consume
the same filesystem.

## Starting and stopping capture

### Steering-wheel control

Press the former encoder-2 push-button (GP19) once to start. Press it again to
stop. This button only controls recording; it does not command vehicle motion.

The wheel display shows:

- `CAP STARTING`: cameras, telemetry, writers, and disk are being checked;
- `RECORDING` and `REC <minutes> <free GB>`: capture is healthy;
- `CAP STOPPING`: MP4 and Parquet files are being finalized;
- `CAP LOW_SPACE` or `CAP FAULT`: recording stopped abnormally.

Do not turn off the Jetson while `CAP STOPPING` is visible. The service needs a
few seconds to finalize both MP4 files and flush Parquet rows.

### Terminal control

From an SSH session on the Jetson:

```bash
sudo systemctl start ethon-v1-capture.service
sudo systemctl stop ethon-v1-capture.service
```

Confirm the current state at any time:

```bash
systemctl is-active ethon-v1-capture.service
```

`inactive` is the correct answer when no capture is running.

The service is manual-only and has `Restart=no`; a crash never silently starts a
new run. Starting it stops camera-conflicting legacy capture/autonomy services.
Stopping it immediately removes the recorder's manual-pedal heartbeat, then
closes the dataset files cleanly.

## Live status and debugging

Follow the recorder log:

```bash
journalctl -u ethon-v1-capture.service -f
```

Press `Ctrl+C` to stop following the log; that does not stop the recorder.

Check service state and the latest failure:

```bash
systemctl status ethon-v1-capture.service --no-pager
journalctl -u ethon-v1-capture.service -n 100 --no-pager
```

Inspect the status shown on the wheel:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
ros2 topic echo /ethon/capture/status
```

The `CYCLONEDDS_URI` line gives debugging commands room to join the already busy
local ROS graph. Without it, a debug command can report that it failed to find a
free participant index even though the vehicle services are healthy. A message
that loopback is not multicast-capable is expected with this local-only setup.

Verify incoming data rates:

```bash
ros2 topic hz /ethon/capture/telemetry
ros2 topic hz /ethon/gps_status
ros2 topic echo /ethon/capture/telemetry --once
ros2 topic echo /ethon/gps_status --once
```

The telemetry topic is a 100 Hz container. Drive velocity fields are populated
at 50 Hz, while current, supply voltage, and fault fields are populated at 20 Hz.
Pedal and steering/CANcoder fields are sampled at approximately 100 Hz.
The deployment check measured the telemetry topic at approximately 100 Hz.

Monitor disk space during a long session:

```bash
watch -n 5 'df -h /home/jetson/ethon; du -sh /home/jetson/ethon/data/raw 2>/dev/null'
```

## Driver event marks

The existing wheel MARK button records a `lap_boundary` event. Additional event
types can be sent from a ROS-enabled shell:

```bash
ros2 topic pub --once /ethon/capture/event std_msgs/msg/String \
  "{data: '{\"event_type\":\"recovery_start\",\"event_value\":\"left\",\"notes\":\"20 cm offset\"}'}"

ros2 topic pub --once /ethon/capture/event std_msgs/msg/String \
  "{data: '{\"event_type\":\"recovery_end\",\"event_value\":\"centered\",\"notes\":\"\"}'}"

ros2 topic pub --once /ethon/capture/event std_msgs/msg/String \
  "{data: '{\"event_type\":\"driver_marked_bad_data\",\"event_value\":\"discard segment\",\"notes\":\"camera bumped\"}'}"
```

## Finding and inspecting a completed run

Runs are stored at:

```text
/home/jetson/ethon/data/raw/YYYY-MM-DD/<run_id>/
```

List the newest runs and inspect metadata:

```bash
find /home/jetson/ethon/data/raw -mindepth 2 -maxdepth 2 -type d \
  -printf '%T@ %p\n' | sort -nr | head

python3 -m json.tool \
  /home/jetson/ethon/data/raw/YYYY-MM-DD/<run_id>/metadata.json
```

Verify that both videos are readable:

```bash
ffprobe -v error -show_entries format=duration,size \
  -of default=noprint_wrappers=1 \
  /home/jetson/ethon/data/raw/YYYY-MM-DD/<run_id>/front_wide.mp4

ffprobe -v error -show_entries format=duration,size \
  -of default=noprint_wrappers=1 \
  /home/jetson/ethon/data/raw/YYYY-MM-DD/<run_id>/front_narrow.mp4
```

Inspect Parquet row counts and schemas:

```bash
python3 - <<'PY'
from pathlib import Path
import pyarrow.parquet as pq

run = Path('/home/jetson/ethon/data/raw/YYYY-MM-DD/<run_id>')
for name in ('frames', 'telemetry', 'events'):
    table = pq.read_table(run / f'{name}.parquet')
    print(name, table.num_rows)
    print(table.schema)
PY
```

In `metadata.json`, require `status: complete`. A `fault` status, non-empty
camera/CAN fault events, sustained dropped-frame flags, or alignment errors over
20 ms means the affected run or interval should be excluded from training.

## Synchronized visual review

The local `viewer/` tool shows both camera streams on one scrubber alongside
steering, speed, pedal, CTRE latency, GPS, and event markers. After downloading
a run, prepare and open it with:

```bash
# On the development PC, from selfdriving/v1/viewer:
python prepare_run.py ../../../jetson/ethon/data/raw/YYYY-MM-DD/<run_id>
npm install
npm run dev
```

Choose the prepared run directory using **Open run folder**. The viewer remains
local so raw vehicle video and telemetry are not published.

## Common failures

| Symptom | Check |
|---|---|
| Wheel shows `CAP FAULT` | `journalctl -u ethon-v1-capture -n 100` |
| Preflight lists `setup_issues` | Fill the run metadata, crop confirmation, and fixed exposure/gain in `v1_capture.json` |
| `pyarrow` missing | `python3 -m pip show pyarrow` |
| Camera startup fault | Ensure `ethon-stack`, legacy `ethon-capture`, and `ethon_display.py` are not using CSI cameras |
| No capture telemetry | Check `ethon-drive.service`, CAN state, and `ros2 topic hz /ethon/capture/telemetry` |
| No GPS heading/fix | Check `ethon-gps.service` and `/ethon/gps_status`; heading is course over ground and needs movement |
| Debug command says no free participant index | Export the `CYCLONEDDS_URI` value shown above, then rerun the command |
| Low-space stop | Move completed runs off the Jetson; do not remove the active run while recording |
| MP4 is incomplete | Stop through the wheel or systemd and wait for finalization; inspect the service log for a forced kill |

## Safety boundary

Capture mode leaves steering neutral for direct hand control. The pedal receives
drive authority only while the recorder publishes a healthy short-lived
heartbeat; recorder or ROS failure removes that authority within 350 ms, followed
by the Phoenix watchdog. This does not replace the physical emergency stop, a
spotter, a clear controlled track, or the low-speed collection procedure.
