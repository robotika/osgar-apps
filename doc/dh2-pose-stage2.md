# Stage 2: first passive articulated pose implementation

Implemented 2026-11-03 in `matty.pose3d`, with historical replay adapter
`dh2.pose_replay`. The existing DH2 controller and robot configuration are
unchanged. Stage 2 remains in progress: deterministic computation is verified;
independent translation and fixed-scene registration are still required.

The [node interface](../matty/README.md) documents frames, required geometry,
clock handling and rejected intervals. Encoder interval checks are shared with
the existing audit in `matty.odometry`. No new dependency versions are needed.

## Run and reproduce

From the repository root:

```bash
uv run --locked python -m dh2.pose_replay \
  /data/matty/m04-dh2-261102_174346.log \
  /data/matty/m04-dh2-261102_175940.log \
  --calibration /data/matty/m04-dh2-oak-calibration.json \
  --output /tmp/dh2-pose-stage2

uv run --locked python -m osgar.replay \
  /tmp/dh2-pose-stage2/m04-dh2-261102_175940/pose.log --module pose
```

Each source gets `pose.log`, `report.json` and `pose.png`. The derived log includes
all node inputs, configuration and four outputs for normal OSGAR replay. Source
status packets are decoded from incoming serial data, preserving packet completion
host time; this is an offline compatibility adapter, not a driver replacement.
Explicit ESP32 reset commands require separate initialized segments.

## Results on existing recordings

All recordings below are M04, 2026-11-02. All six derived logs pass normal
`osgar.replay --module pose`, with all four output streams matching and zero
reported scheduling delay.

| Recording | Setup | Poses / integrated intervals | Final joint XY (m) | Median constraint RMS (mm/update) |
| --- | --- | --- | --- | --- |
| `174346` | Flat | 165 / 161 | 0.0013, −0.0035 | 0.378 |
| `174419` | Front right on box | 201 / 197 | 0.0012, −0.0021 | 0.443 |
| `174456` | Both front wheels on box | 213 / 209 | −0.0015, −0.0492 | 0.431 |
| `174534` | Rear right on box | 200 / 196 | −0.0020, 0.0008 | 0.829 |
| `175816` | First box approach | 523 / 519 | 0.9415, 0.0077 | 0.492 |
| `175940` | Second box approach | 296 / 292 | 0.9109, −0.0390 | 0.397 |

These are model outputs, not measured displacement errors. In particular the
49 mm lateral final position in `174456` needs scene comparison; do not interpret
it as proven physical movement. Small constraint RMS does not exclude common-mode
wheel slip or incorrect geometry.

Each log skips 73–74 startup statuses before synchronized OAK coverage; no tail
statuses remain pending. Invalid intervals include encoder/power faults, and
wheel jumps in `175816`. Every run ends with `translation_continuous=false`:
rejected travel is unknown, including shutdown faults, rather than silently zero.

Synthetic tests cover signed travel, counter wrap/fault recovery, articulation
with zero mean wheel travel, turning/yaw wrap, independent section roll, housing
lever arm, quaternion interpolation and batch timing. The CRC audit test now
asserts rejection/resynchronization without requiring a particular core logging
mechanism. The package also builds offline with the new `matty` module included.

## Geometry and timing still provisional

The adapter composes the reviewed raw-to-fused axis rotation, factory IMU-to-left
rotation, left rectification and a nominal optical-to-body mounting. The factory
file hash and device ID identify this assumption set in output metadata.
It is specific to the reviewed M04 installation; it is not a verified mount
calibration. Rear ESP mounting defaults to identity after the driver's OSGAR
angle conversion. Front and rear yaw origins are aligned independently once.

Housing offset `[0.21, 0, 0.16]` relative to the joint uses measured dimensions.
The rear axle distance 0.16 m, track 0.315 m and wheel order FL/FR/RL/RR are still
provisional firmware-derived settings. Joint Z is assumed, not observed.
The housing reference must not be passed directly to depth projection as an
individual camera optical centre.

First-batch clock alignment is approximate; it does not establish exact depth/IMU
synchronization. No accelerometer double integration is introduced.

## Next step

Compare static surfaces across depth frames using this pose, explicit rectified
optical extrinsics and reviewed depth intrinsics/timestamps. Quantify residuals
during stationary articulation before using translation trials. Confirm wheel
order and rear mounting physically, then perform measured forward/reverse motion.
Only after those checks should local-map aggregation or driving decisions consume
the provider. See the [stage 2 plan](dh2-stage2-plan.md) for acceptance criteria.
