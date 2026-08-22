# Self-driving v1: model and data capture

## Goal

Self-driving v1 only needs to steer around a fixed, empty track in one direction at low speed. It does not need obstacle avoidance, route selection, traffic handling, throttle control, or general road-driving behavior.

The recommended policy is a small temporal imitation-learning model:

> **ImageNet-pretrained ResNet-18 + a small GRU + a one-second curvature trajectory head**

A VLA, Alpamayo, ACT, or a full AutoE2E stack would add compute and adaptation work without helping this single-behavior task. NVIDIA's original DAVE-2 work successfully learned steering from one front camera, represented steering as curvature `1/r`, sampled training frames at 10 Hz, emphasized curves, and added recovery examples. This design modernizes that recipe with pretrained weights and short temporal context. See the [NVIDIA DAVE-2 paper](https://arxiv.org/abs/1604.07316).

## Model contract

Use only the **front-wide camera** for v1. Record the front-narrow camera too, but do not train with it initially.

### Inputs

At each inference step, provide:

- Four RGB images from the front-wide camera:
  - `t - 300 ms`
  - `t - 200 ms`
  - `t - 100 ms`
  - `t`
- Vehicle speed in metres per second.
- Current steering-shaft position.
- Current steering-shaft rate.

Preprocess each image as follows:

1. Record at `1280x720`.
2. Apply a fixed crop that removes irrelevant sky and visible chassis.
3. Resize to `320x180` without changing the aspect ratio.
4. Normalize using the ResNet/ImageNet normalization.
5. Never change the crop after collecting the main dataset.

### Network

```text
4 images
   |
   v
shared pretrained ResNet-18
   |
   v
4 x 512-dimensional visual features
   |
   +-- vehicle speed
   +-- steering-shaft position
   +-- steering-shaft rate
   |
   v
GRU, hidden size 256
   |
   v
MLP
   |
   v
10 future curvature values
```

The output is:

```text
kappa(t+0.1), kappa(t+0.2), ..., kappa(t+1.0)
```

`kappa` is signed curvature in `1/metres`, not steering-motor voltage or raw shaft angle. ResNet-18 has readily available pretrained weights and straightforward Jetson deployment. See the [Torchvision ResNet documentation](https://docs.pytorch.org/vision/stable/models/resnet).

### Curvature-to-steering conversion

The steering controller converts predicted curvature into a steering-shaft target:

```text
equivalent wheel angle = atan(1.52 m * kappa)
                       |
                       v
measured wheel-angle <-> shaft-angle calibration
                       |
                       v
steering Kraken position request
```

The physical linkage produces the differential left/right Ackermann angles. Software should use a measured curvature-to-steering-shaft lookup or fitted function rather than relying only on the theoretical linkage dimensions.

Use the Talon FX's onboard position loop. CTRE states that its internal closed-loop controller operates at 1 kHz. See the [CTRE closed-loop documentation](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html).

## Capture and runtime rates

| Signal | Raw capture rate | Training/runtime use |
|---|---:|---|
| Front-wide video | 30 FPS | Training sequences sampled at 10 Hz |
| Front-narrow video | 30 FPS | Record only; unused initially |
| CANcoder position and velocity | 100 Hz | Interpolate to image timestamps |
| Steering Kraken state and command | 100 Hz | Interpolate to image timestamps |
| Three drive-motor velocities | 50 Hz | Convert to vehicle speed |
| Drive currents, voltage, and faults | 20 Hz | Diagnostics |
| Accelerator ADC | 100 Hz | Record; not a model target |
| GPS position, heading, and fix | Native rate, likely 5-10 Hz | Evaluation and coarse location |
| Model inference | 20 Hz | Receding-horizon prediction |
| New steering setpoint | 20 Hz | Use the latency-adjusted trajectory point |
| Talon FX position loop | Internal 1 kHz | Actuator control |

Record images at 30 FPS so the raw dataset retains all available information, but generate training examples at 10 Hz. Adjacent 30 FPS frames are highly redundant; DAVE-2 similarly sampled collected video at 10 FPS for training.

Configure the important CTRE position and velocity signals to 100 Hz and reduce unused signal rates. CTRE warns that increasing every signal can saturate CAN and recommends keeping bus utilization below 90%. See the [CTRE status-signal documentation](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html).

## Synchronization requirements

Every sample must use the Jetson's monotonic clock.

- Timestamp images at frame exposure/capture time, not after inference or encoding.
- Timestamp every CAN sample when received, retaining the CTRE device timestamp when available.
- Aim for less than `10 ms` image-to-steering alignment error.
- Reject or flag samples exceeding `20 ms` alignment error.
- Interpolate CANcoder position and velocity to the exact camera timestamp.
- Store capture latency and dropped-frame counters.
- Do not use Raspberry Pi side-camera frames in the v1 model. Their network delay and clock synchronization add unnecessary failure modes.

Before training, measure the camera-to-steering latency using the logs. At runtime, select the predicted curvature point corresponding to the measured total delay rather than always taking `kappa(t+0.1)`.

## Recording format

Use one directory per uninterrupted run:

```text
data/raw/<date>/<run_id>/
|-- metadata.json
|-- front_wide.mp4
|-- front_narrow.mp4
|-- frames.parquet
|-- telemetry.parquet
`-- events.parquet
```

This follows the same general structure as the [LeRobot dataset format](https://github.com/huggingface/lerobot): synchronized MP4 video with Parquet state and action data. LeRobot's recording and dataset tooling may therefore be reusable even though v1 uses a custom steering policy.

### Run metadata

`metadata.json` should include:

- Run ID and UTC start time.
- Git commit and software version.
- Camera identity and configuration.
- Resolution and fixed crop.
- Exposure, gain, focus, and white balance settings.
- Camera calibration hash.
- Steering calibration hash.
- Wheelbase and reduction configuration.
- Track name, configuration, and direction.
- Driver identifier.
- Weather and lighting.
- Nominal speed.
- Any known faults or anomalies.

### Frame records

Each row in `frames.parquet` should contain:

```text
timestamp_ns
front_wide_frame_index
front_narrow_frame_index
dropped_frame_flags
capture_latency_ms
```

### Telemetry records

Each row in `telemetry.parquet` should contain:

```text
timestamp_ns
cancoder_position_rad
cancoder_velocity_rad_s
steering_motor_position
steering_motor_velocity
steering_target
steering_voltage
steering_current
drive_1_velocity
drive_2_velocity
drive_3_velocity
vehicle_speed_m_s
pedal_fraction
gps_latitude
gps_longitude
gps_heading
gps_fix_quality
manual_or_auto
estop
can_faults
```

### Event records

Each row in `events.parquet` should contain:

```text
timestamp_ns
event_type
event_value
notes
```

At minimum, record these event types:

- Recording start and stop.
- Lap boundary.
- Manual-to-autonomous transition.
- Autonomous-to-manual takeover.
- Deliberate recovery start and completion.
- Emergency stop.
- Camera or CAN fault.
- Driver-marked bad data.

Store raw sensor values. Derive curvature labels in preprocessing so improved steering calibration does not require recollecting data.

## Initial collection plan

Target approximately **60-90 minutes** before training the first model:

| Collection type | Duration | Purpose |
|---|---:|---|
| Smooth, centered laps | 35-45 minutes | Learn ordinary track following |
| Deliberate recovery demonstrations | 20-30 minutes | Learn to return from policy drift |
| Speed and lighting variation | 10-15 minutes | Reduce sensitivity to minor operating changes |

For the fastest first result:

- Use one fixed track configuration.
- Drive only the intended direction.
- Use a speed governor, initially around walking speed.
- Keep the camera mount and fixed crop unchanged.
- Lock camera focus.
- If exposure remains automatic, log exposure and gain.
- Exclude runs with camera motion, loose steering components, CAN faults, or significant dropped frames.

At 10 training samples per second, 75 minutes produces about 45,000 candidate sequences.

## Recovery data

A dataset containing only perfect centered laps teaches the car what to do while centered, but not how to return after its own small error. DAVE-2 identified this distribution-shift problem and augmented off-centre positions with corrected steering labels.

During recovery runs, begin from controlled perturbations such as:

- `20-40 cm` left or right of the desired path.
- Approximately `+/-5 degrees` heading error.
- Mildly excessive steering entering a curve.
- Mildly insufficient steering entering a curve.

The human should recover smoothly rather than snapping to the centre. Perform recovery collection only at very low speed with the track clear.

After training the first model:

1. Run it with a human ready to take over.
2. Record every takeover and the complete subsequent correction.
3. Add those segments to the dataset with `3-5x` sampling weight.
4. Retrain.
5. Repeat until the recurring failure modes disappear.

This is effectively DAgger-style corrective collection. LeRobot provides [human-in-the-loop DAgger machinery](https://github.com/huggingface/lerobot/blob/main/docs/source/hil_data_collection.mdx) that can be adapted to custom hardware.

## Dataset preprocessing

For each 10 Hz training timestamp `t`:

1. Select the nearest valid front-wide frames at `t-300`, `t-200`, `t-100`, and `t` milliseconds.
2. Reject the sample if any required image is missing or outside the timestamp tolerance.
3. Interpolate steering and speed telemetry to `t`.
4. Convert the measured steering-shaft positions from `t+0.1` through `t+1.0` into curvature using the versioned steering calibration.
5. Save the ten-value future-curvature target.
6. Attach flags for curve severity, recovery, intervention, lighting, and data quality.

The preprocessing output should retain the raw timestamps and calibration versions used to create every label.

## Dataset split

Use:

- `70%` training.
- `15%` validation.
- `15%` test.

Split by complete run or complete lap, never by randomly assigning nearby frames. Random frame splitting would place almost identical adjacent images in training and validation and produce misleadingly good metrics.

Keep at least one complete recording session or lighting condition exclusively for the test set.

## Sampling balance

Keep all raw data, but balance training batches across:

- Straight driving.
- Moderate left turns.
- Sharp left turns.
- Moderate right turns.
- Sharp right turns.
- Recovery demonstrations.
- Human interventions.

Do not let straight driving dominate simply because it occupies most recorded frames. Upweight recoveries and interventions rather than permanently deleting ordinary data.

## Training configuration

Recommended starting configuration:

| Setting | Initial value |
|---|---:|
| Optimizer | AdamW |
| Batch size | 32 |
| Trajectory-head learning rate | `3e-4` |
| ResNet backbone learning rate | `1e-5` |
| Epochs | 30-50 with early stopping |
| Main loss | Huber loss on future curvature |
| Temporal head | GRU with hidden size 256 |
| Output horizon | 1 second |
| Output interval | 100 ms |

Weight the first 500 ms of the predicted trajectory more heavily than the distant portion. Add a small smoothness penalty on differences between consecutive predicted curvature values.

### Augmentation

Safe initial augmentations:

- Brightness changes.
- Gamma and contrast changes.
- Mild blur.
- Mild sensor noise.
- Mild video-compression artifacts.
- Small exposure and colour-temperature changes.

Avoid initially:

- Horizontal image shifts without mathematically correcting curvature.
- Random crops that alter camera geometry.
- Heavy perspective transformations.
- Horizontal flips unless curvature signs are inverted and the resulting scene remains physically plausible.

## Deployment

1. Train off the vehicle on a workstation or cloud GPU.
2. Export the model to ONNX.
3. Build a fixed-shape TensorRT FP16 engine for four `320x180` images plus vehicle state.
4. Benchmark on the Jetson under the same camera, CAN, logging, and thermal load used during driving.
5. Require a sustained 20 Hz inference rate with bounded latency and no memory pressure.
6. Fall back to neutral/safe steering behavior if inference output is late, invalid, non-finite, or outside calibrated limits.

## Evaluation

Offline steering error is not the real success criterion. Evaluate in this order:

1. **Held-out data:** measure median, mean, and 95th-percentile curvature error.
2. **Shadow mode:** run the model while a human drives; log predictions without applying them.
3. **Low-speed rollout:** begin at half the data-collection speed.
4. **Corrective collection:** record and retrain on interventions.
5. **Acceptance run:** complete repeated full laps at the target v1 speed.

V1 is complete when the car can:

- Finish 10 consecutive laps at the fixed test speed.
- Require zero interventions.
- Remain inside the defined track boundary.
- Avoid steering saturation and sustained oscillation.
- Remain within calibrated steering position and rate limits.
- Enter its safe state on stale images, stale CAN data, model timeout, or invalid model output.

## Safety boundary

This document specifies steering-model data capture, not a complete safety system. Autonomous rollout still requires a reliable stopping method, independent emergency stop, watchdog, speed limit, steering limits, and a clear controlled track.
