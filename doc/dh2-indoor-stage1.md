# DH2 stage 1 — indoor sensor and geometry recordings

Use M04 indoors to collect the data needed for `MattyPose3D`. No map veto or new
pose estimator is enabled yet. The shared `dh2/config/dh2.json` now enables OAK
integrated orientation and raw accelerometer/gyro, with batches of five samples.
The existing driving algorithm and emergency-stop default are unchanged.

**Status at the end of 2 November: stage 1 complete.** Factory calibration,
verified raw/fused IMU capture, front/rear posture tests and box-contact regression
recordings are available. The next work is the [stage 2 plan](dh2-stage2-plan.md),
starting with a four-encoder audit before choosing the articulated pose model.

The [first three indoor recordings](dh2-indoor-261102.md) provide good integrated
orientation/depth data and useful tilt/articulation checks. Raw IMU streams were
not registered despite the config flags; verify the interpreter/driver used on
the robot before relying on raw capture. The operator identified and corrected
a different OAK branch; the next recording should verify raw output. STOP-held startup is optional when
physical access requires normal lid closure and zero-speed look-around.

The [later raw-enabled set](dh2-indoor-261102-raw.md) now verifies all three IMU
streams and supplies a tested raw/fused axis mapping for the next pose-provider
stage. Acceleration scale and the mechanical camera-to-chassis transform still
need validation.

The subsequent [two low-box approach trials](dh2-box-261102.md) capture the box
before its near-field disappearance and confirm front contacts in both runs.
They also require checking stationary articulation sweeps in the map design.

## Stages

1. **Now:** factory calibration, stationary/scan/short-motion recordings, sensor
   timing/validity checks and mechanical measurements.
2. **Next:** reusable front-frame pose3d provider. Check a fixed wall stays fixed
   during stationary articulation and a measured straight displacement.
3. **Then:** rolling map in shadow mode, periodic snapshots and viewer overlay;
   test a measured low obstacle and flat-floor negative controls indoors.
4. **Finally:** enable the short-sweep motion veto and validate it physically,
   including the outdoor terrain/step case when the robot is available there.

## 1. Save camera factory calibration

Stop any other process using the camera. From the repository root:

```bash
uv run --locked python -m dh2.capture_calibration \
  --output /tmp/dh2-oak-calibration.json
```

This opens only the OAK device, reads EEPROM and queries left-camera intrinsics,
rectification rotation and IMU-to-left extrinsics with explicit metre units. It
records the device ID and full factory data. Missing optional extrinsics are
reported in `unavailable`, not silently replaced with identity transforms.
Save this file with the recordings. It is not a robot motion command.

Camera intrinsics queried at 640×400 are labeled as resized camera intrinsics;
we still need to derive/verify the actual rectified depth projection. Factory
calibration also does not measure the camera-to-front-chassis mount.

## 2. Stationary IMU baseline — STOP stays pressed

Put the robot on a level floor facing a textured wall approximately 1–2 m away.
Keep STOP pressed for this first recording. From `dh2/`:

```bash
PYTHONPATH=.. ../.venv/bin/python -m osgar.record config/dh2.json \
  --duration 30 --note 'DH2 stage1 stationary, STOP held, level floor' \
  --params app.terminate_on_stop=False app.max_speed=0 app.allow_reverse=False
```

`terminate_on_stop=False` permits this STOP-held data collection; the shared
config still terminates on STOP by default. There should be camera frames and
all three OAK IMU streams despite the lack of motion. OSGAR controls duration.
Keep the existing `OSGAR_LOGS_PREFIX` and log-directory setup.

Optionally make another STOP-held recording with the front section held at a
known small pitch/roll for several seconds, returning to level between poses.
Note which side was raised and the approximate angle/height. This helps identify
axis signs; it is not yet a calibrated body orientation estimate.

## 3. Stationary articulation — propulsion disabled

Return the robot to the floor and release STOP before recording:

```bash
PYTHONPATH=.. ../.venv/bin/python -m osgar.record config/dh2.json \
  --duration 45 --note 'DH2 stage1 zero-speed articulation facing fixed wall' \
  --params app.max_speed=0 app.allow_reverse=False
```

Zero propulsion still allows Matty to look around. The existing controller scans
at ±30 degrees. Capture a fixed wall/corner so we can test the front camera's
rotation **and translation** while the platform articulates. The raw orientation
is not yet aligned to the front-body frame, so a changed Euler axis is not itself
an error. Pressing STOP ends this run normally.

## 4. Optional short movement on clear level floor

For a small existing-controller movement, with STOP released:

```bash
PYTHONPATH=.. ../.venv/bin/python -m osgar.record config/dh2.json \
  --duration 20 --note 'DH2 stage1 short floor movement; measure actual displacement' \
  --params app.max_speed=0.1 app.min_speed=0.1 app.explore_speed=0.1 \
  app.step_distance=0.3 app.max_travel=0.3 app.allow_reverse=False
```

This allows motion at up to 0.1 m/s, with a 0.3 m **odometry** travel cap. It is
still the existing exploration controller, not an exact straight-distance command;
obstacles/detections can alter the sequence. Use a clear scene without cones and
measure actual displacement/heading of a mark fixed to the front section. A
straight segment is useful for pose validation; a stopped or turning run is still
useful sensor data. Do not use this stage to drive into a test step: map protection
has not been implemented.

Record camera height, its forward/lateral position relative to a reproducible
front-section mark, joint/axle positions, both section lengths/widths and bumper
extent. Note where the ESP32 IMU is mounted. These measurements resolve the pose
origin/lever-arm question; factory camera calibration alone cannot.

## Check each resulting log

From the repository root, substituting its actual filename:

```bash
uv run --locked python -m dh2.sensor_check /data/matty/RECORDING.log \
  --output /tmp/dh2-sensor-check --require-oak-imu
```

The command saves `report.json` and `sensors.png`. It exits nonzero if required
streams are empty, samples are malformed, IMU device timestamps reset/repeat or
have gaps over 100 ms, or quaternion norms are invalid. Camera payloads are counted
without decoding video. OAK sample rate should be close to the configured 100 Hz,
with individual device timestamps within batches; inspect the reported rate and
maximum gap, especially during articulation.

Plots show OAK orientation/acceleration/gyro on device time, and ESP32 attitude,
articulation and planar odometry on host-log time. **Those axes and clock epochs
are not yet aligned.** Receipt-minus-device variation is only a diagnostic spread,
not absolute sensor latency. A capture check passing does not validate camera
geometry, pose3d accuracy or collision avoidance.

For historical contest logs, omit `--require-oak-imu` to generate a report despite
missing OAK IMU. The [map proposal](dh2-local-map-mvp.md) describes the next stages.
