# Self-driving v1 data capture firmware

This package implements the recorder specified in
`selfdriving/v1/DATACAPTURE.md`. It is intentionally a manual service: it does
not start at boot and systemd does not restart it after a failure.

## What a run records

Each uninterrupted run is written under:

```text
/home/jetson/ethon/data/raw/YYYY-MM-DD/<run_id>/
├── metadata.json
├── front_wide.mp4
├── front_narrow.mp4
├── frames.parquet
├── telemetry.parquet
└── events.parquet
```

Both CSI streams use Jetson hardware H.264 encoding. The wide-camera frame is
the synchronization anchor; each row records its nearest recent narrow frame,
per-camera latency, detected drops, alignment error, and a validity flag. Drive
telemetry is sampled in the existing drive process so there is only one Phoenix
owner for each CAN device.

Capture rates are 100 Hz for CANcoder/steering/pedal, 50 Hz for drive velocity,
20 Hz for currents/voltage/fault fields, and the GPS receiver's native rate.
All primary timestamps use `time.monotonic_ns()` on the Jetson. The estimated
Phoenix source time and reported transport latency are retained separately.

## Driver controls and feedback

The former encoder-2 push-button on GP19 is the dedicated start/stop toggle.
It asks systemd to start or gracefully stop `ethon-v1-capture.service`. The
wheel display reports:

- `CAP STARTING` while cameras and telemetry are being verified;
- `RECORDING` and `REC <minutes> <free GB>` during capture;
- `CAP STOPPING` while MP4 and Parquet files are finalized;
- `CAP LOW_SPACE` or `CAP FAULT` when capture stops abnormally.

The existing MARK button writes a lap-boundary event. Additional events can be
sent to `/ethon/capture/event` as JSON containing `event_type`, optional
`event_value`, and optional `notes`. Supported intended values include
`recovery_start`, `recovery_end`, and `driver_marked_bad_data`.

The display considers a missing recorder status heartbeat a fault. Starting a
recording does not command motion. Manual pedal authority is enabled only after
both cameras and high-rate CAN telemetry are alive, and is renewed by a 10 Hz
heartbeat. If the recorder or ROS graph dies, the drive node removes authority
after 350 ms and the Phoenix unmanaged watchdog then disables drive output.
Steering remains neutral/released for hand steering during manual capture.

## Storage policy

The default configuration records both cameras at 8 Mbit/s. Expected storage is:

| Duration | Video + telemetry | Guarded capture allowance | Required free space including 10 GB reserve |
|---:|---:|---:|---:|
| 60 min | 7.45 GB | 9.31 GB | 19.31 GB |
| 90 min | 11.18 GB | 13.97 GB | 23.97 GB |

The service refuses to start unless the full 90-minute allowance plus reserve
is free. During a run it stops cleanly before free space falls below 10 GB.
These are decimal GB, matching disk-tool output.

It also refuses to start until track/driver/weather/lighting/nominal-speed
metadata is filled in, the fixed crop is explicitly confirmed, and fixed
camera exposure/gain ranges are configured. Those values require an operator
and a short hardware tuning session; leaving silent placeholders would make the
first dataset difficult to reproduce.

The final Jetson preflight on 2026-08-22 found 40.17 GB free, which exceeds the
90-minute requirement by 16.20 GB. ROS, Phoenix, PyGObject, PyArrow 25.0.1 with
Zstandard, and every required GStreamer element were present. The capture
telemetry topic measured approximately 100 Hz, and refreshed CTRE status signals
reported roughly 10 ms latency against the recorder's 20 ms activation gate.

## Known hardware validation item

The current firmware identifies front-wide as sensor 1 (IMX477 fisheye) and
front-narrow as sensor 0 (IMX219). The proven narrow-camera mode runs at 28 FPS,
not the 30 FPS target in the data plan, so the local config retains 28 FPS.
During deployment, enumerate Argus modes and test whether this exact camera can
produce 1280x720 at 30 FPS reliably. Do not silently interpolate 28 FPS video
to 30 FPS.

## Deployment and first-run preflight

The Jetson needs ROS 2 Humble, Phoenix 6, PyGObject/GStreamer with NVIDIA camera
and H.264 plugins, and PyArrow. Before the first drive:

1. Copy the local firmware and write the deployed Git commit to
   `/home/jetson/ethon/.firmware-version`.
2. Install the service without enabling it. Install
   `ethon_capture_control.sh` root-owned and mode 0755 as
   `/usr/local/sbin/ethon-capture-control`, then install and validate the
   audited sudo rule.
3. Update the wheel Pico's `code.py` so GP19 emits `capture_toggle`.
4. Run `python3 v1_data_capture.py --preflight`.
5. Verify both camera modes, CAN rates/utilization, GPS heading/fix quality, and
   at least 23.97 GB free for the default 90-minute limit.
6. Make a stationary 30-second capture, stop it from the wheel, and inspect all
   six outputs before collecting driving data.

Steps 1-5 are deployed. The service is static/inactive by design, and GP19's
updated CircuitPython firmware is running with the previous `code.py` preserved
on-device as `/code_pre_v1_20260822.py`. Step 6 remains blocked until the
operator fills the required run metadata, confirms the fixed crop, and enters
fixed exposure/gain ranges for both cameras. The indoor GPS check reported no
fix, so GPS fix and heading must also be verified outdoors before driving.
