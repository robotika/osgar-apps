# Passive Matty pose provider

`matty.pose3d:MattyPose3D` is a reusable shadow node. It publishes estimates and
quality information, with no motion commands. This first version has planar
joint translation and separate front/rear 3D attitudes; it is not full terrain
odometry and is not used by the DH2 controller yet.

Inputs:

| Input | Meaning |
| --- | --- |
| `odometry` | `platform.odometry`: `[status_bits, speed_mm_s, joint_cdeg, [four cumulative uint16 wheel distances in mm]]` |
| `rotation` | `platform.rotation`: ESP32 yaw/pitch/roll in OSGAR centidegrees |
| `orientation_list` | `oak.orientation_list`: batches of `[device_seconds, accuracy, x, y, z, w]` |

The bundled odometry requires the OSGAR `feature/matty-encoders` driver change.
Historical logs can instead be decoded with `dh2.pose_replay`. There is no
dependency lockfile update to an unmerged OSGAR branch.

Outputs use OSGAR `[xyz, quaternion_xyzw]`:

| Output | Reference |
| --- | --- |
| `pose3d` | Central joint position, front-section attitude |
| `rear_pose3d` | Same joint position, rear-section attitude |
| `housing_pose3d` | Nominal camera housing reference, front-section attitude |
| `pose_quality` | Source time, timing/geometry provenance, rejected intervals, model disagreement and limitations |

The origin is the joint at the first synchronized sample. Initial front/rear
yaw is set to ±half the joint angle; subsequent attitudes use fixed initial
world alignments. Front/rear roll and pitch remain separate. Joint Z stays zero;
nominal floor Z is −0.13 m for the measured joint height. Housing Z includes its
rotated lever arm, but does not measure joint heave.

Translation fits four wheel rolling constraints and two axle lateral constraints
using front/rear yaw and finite axle lever-arm changes. Thus stationary steering
can move the joint even when mean wheel travel is zero. This is a planar no-slip
model: constraint RMS reports disagreement, not a complete slip detector.

Required configuration (distances in metres):

- `wheel_order`: incoming indices for FL, FR, RL, RR.
- `front_axle`, `rear_axle`, `track`: positive axle-to-joint distances and track.
- `housing_offset`: housing reference relative to joint in front forward/left/up.
- `front_from_fused`, `rear_from_esp`: XYZW rotations mapping each sensor frame
  into its section body frame. Sensor attitudes are right-multiplied by the
  inverse mounting rotation.

Optional settings: `max_gap=0.5` s, `max_wheel_speed=1.5` m/s,
`max_attitude_gap=0.3` s, `max_rear_age=0.15` s, `imu_time_offset`,
`geometry_id`, `geometry_verified=false`.

OAK device sample times are mapped to host-log seconds by a configured constant
offset, or approximately by first batch receipt minus its last device timestamp.
Each sample keeps its individual acquisition time. Front attitude uses SLERP
with bracketing samples; rear attitude is the latest preceding status attitude.
Pending statuses await a camera bracket. Startup samples without coverage are
counted as skipped. A nonincreasing OAK acquisition time raises an error requiring
a new clock alignment rather than silently treating a camera reboot as motion.

Encoder faults, preceding encoder faults, power faults, excessive gaps and
implausible wheel increments reject translation and rebase wheel counters.
Unknown travel permanently clears `translation_continuous` for that run.
All outputs currently carry `usable_for_driving=false`. Geometry verification
alone does not enable driving. The housing reference is **not** a calibrated
rectified optical frame.

See [DH2 replay findings and remaining validation](../doc/dh2-pose-stage2.md).
