# DH2 stage 2 — articulated Matty pose and odometry

End-of-day handoff, **2 November 2026**. Stage 1 is complete. This document plans
stage 2; it does not enable a pose estimator, change steering, or claim that slip
or six-degree-of-freedom localization is solved.

## Starting point and scope

Stage 1 produced factory calibration, verified fused/raw camera IMU recording,
repeatable flat/front/rear support experiments, and two low-box contact cases.
The latest datasets replay normally. Findings and figures are linked below.

**Stage 2 deliverable:** a reusable Matty pose provider in osgar-apps, with explicit
front/rear frames, timestamped encoder-based motion, camera attitude conversion,
and recorded estimation-quality information. Develop it offline first, then run
in shadow mode. Keep `platform.pose2d` and existing controller behavior intact.

The first implementation task is an **offline four-encoder audit**, before
choosing a replacement translation model. Mapping, periodic grid snapshots and
motion vetoes follow in stage 3; they must use both translating and zero-speed
articulation sweeps.

## Agreed geometry and frame decisions

Operator measurements, with the robot straight and level:

| Quantity | Measurement | Interpretation / remaining check |
| --- | ---: | --- |
| Camera housing reference height above floor | 0.29 m | Operator clarified these measurements refer to the housing, not an individual lens |
| Joint centre height above floor | 0.13 m | Nominal level posture, not a fixed world altitude on uneven terrain |
| Camera forward of joint | 0.21 m | Rigid front-section lever arm, not a world-X offset while turning |
| Front axle forward of joint | 0.16 m | Camera is approximately 0.05 m ahead of front axle |
| Camera above joint | 0.16 m | Derived as 0.29−0.13 m |
| Camera housing lateral offset | 0 m | Confirmed centred; internal left/RGB/right lenses and IMU retain separate factory offsets |

The nominal joint-to-camera-housing reference vector is therefore
**`[0.21, 0, 0.16] m`** in front forward/left/up axes. The operator confirmed the
housing is centred on 3 November. It contains three cameras; this vector must not
be assigned directly to every optical centre. Define the housing reference point
precisely and relate it to a factory camera reference before composing the
left/RGB/right and IMU offsets. The stereo baseline/IMU translations remain part
of the transform chain.

Use a common **central joint position** `p_J`, with separate **front and rear
orientations** `q_F` and `q_R`. Do not give the whole articulated robot one
averaged roll/pitch. Confirm whether the physical articulation axes share one
centre; record offsets if the single-pivot approximation is insufficient.

Define front/rear frames with origins at that joint reference and axes fixed to
the respective sections: X forward, Y left, Z up. They can share a position while
having different orientations. The camera-to-front transform is constant; the
camera-to-rear transform is not. For a point fixed to the front:

```text
p_camera = p_J + R(q_F) * t_front_camera
R_odom_camera = R(q_F) * R_front_camera
```

Define run-local `odom` with origin at the initial joint position, gravity-aligned
vertical and a recorded arbitrary initial heading. In this convention initial
joint Z=0, nominal level floor Z=−0.13 m, and the camera reference Z=+0.16 m.
Do not add the 0.13 m ground height twice. An alternative floor-origin mode would
need explicit different metadata; do not mix conventions silently.

**Unresolved:** the point/heading that existing `platform.pose2d` actually represents.
It is a legacy estimate to compare against, not joint-centre ground truth.

## 2A. Offline encoder and protocol audit — first work item

The [first audit implementation and findings](dh2-encoder-audit.md) are available
as `dh2.encoder_audit`. Seven existing runs were processed, with ten focused
tests and an independent 370-packet comparison against the new driver output.
Read-error flags and acquisition/receipt timing need handling before pose
integration; physical wheel-ID confirmation remains outstanding.

### Encoder output prepared on 3 November

The OSGAR `feature/matty-encoders` checkout now has a small proposed driver change:
`platform.encoders` publishes a **list of four integers** on every status packet.
Values remain unsigned cumulative wheel distances in millimetres, in firmware
order, with the existing 16-bit wrap representation. No unwrapping, wheel relabeling
or odometry-model change is performed by the driver. The output is appended to
the registered outputs so existing registration order is preserved.

Both legacy and attitude-bearing packets are covered, including unsigned boundary
values and wrap transitions. All 14 Matty driver tests pass. A forced platform
replay of box run `175940` produces 370 encoder messages; comparing per-stream
payload sequences shows all ten original platform outputs unchanged. Ordinary
platform replay against the old file rejects the newly registered output because
it was not recorded; forced replay is required to extract it. This is distinct
from the unchanged baseline controller replay.

This work is committed in the separate OSGAR checkout as **`ce79024b`** on
`feature/matty-encoders`. The osgar-apps dependency pin is unchanged. Use the intended checkout explicitly
when testing/deploying; updating a checkout alone does not replace an installed
driver in another environment. Future runtime consumers can subscribe directly;
historical recordings still require the packet audit below.

### Source facts to verify against the robot

The inspected OSGAR driver (`osgar.platforms.matty`) parses four `uint16` fields
at payload offset 12, then ignores them for its planar integration. It integrates
reported speed with a fixed 0.1 s interval and joint-dependent curvature using a
0.32 m axle-distance constant.

Available Matty firmware source, snapshot **`2d64566`** in `~/git/matty`, shows:

- `esp/Robot/src/robot.cpp`, `updateEncoder`: accumulates signed wheel increments;
  even-indexed wheel signs are reversed for the left side.
- `updateOdometry`: reported speed is the **average of four signed wheel
  increments**, scaled by wheel distance-per-step and the firmware time interval.
- `esp/Robot/src/communication.cpp`: sends accumulated encoder distances scaled
  to millimetres in four `uint16` fields. They are not simply raw servo counts.
- `esp/matty-v8.txt`: documents millimetres and the 26-byte attitude-bearing status
  payload. The source contains older 20-byte protocol support too.

These are facts about the available source, **not proof that this exact revision
is flashed on M04**. Check packet/version evidence and observed increments. Do
not reapply the servo-to-wheel scale to values already transmitted in millimetres.
Firmware speed and packet encoder deltas may cover different intervals; this is
particularly relevant during look-around and intermittent contact.

### Implement and inspect

1. Read the incoming wire stream feeding `platform.esp_data` (`serial.raw` in
   the DH2 config), not the outgoing `platform.esp_data` command stream. Reuse
   the OSGAR framing/checksum and
   escaping rules; packets can span messages or multiple packets can share one.
   Do not assume one log message is one complete status packet.
2. Decode valid incoming status packets; distinguish acknowledgements, version,
   GPS and outbound/control traffic. Preserve receive time, packet counter, flags,
   speed, joint angle, four wheel distances and optional ESP32 attitude.
3. Establish physical wheel order/servo IDs, signs and counter semantics. Test
   forward, reverse, one-wheel changes and wrap crossings. If these are modulo
   65536 mm counters, use signed modular differences only when the displacement
   bound makes the result unambiguous. Detect resets/gaps rather than treating
   every discontinuity as travel. Do not assume a packet counter counts every
   control/ack message until the protocol is checked.
4. Compare `(sum(delta_wheel_mm)/4)` with integrated reported speed over the same
   receive intervals, and compare both with legacy pose increments. Use actual
   time separation; identify firmware/sample timing and quantization effects
   instead of changing a scale to hide them.
5. Plot each wheel, front/rear means, left/right differences, joint angle/rate,
   OAK angular velocity, ESP32 attitude, commands and bumper transitions.
   Look-around is expected to produce different/opposite wheel motion: unequal
   encoders alone are not evidence of slip.
6. Produce machine-readable diagnostics and a short conclusions note identifying
   what can be recovered reliably from current logs and what remains ambiguous.

**First dataset:** four raw-enabled posture runs and the two indoor box runs.
Then inspect the final part of event `101546` with its older IMU limitations.

**Exit:** verified packet semantics, wheel labels or explicitly documented unknown
labels, robust reset/wrap handling, and a measured comparison with the existing
speed integration. No change to old recorded/replayed outputs.

## 2B. Resolve attitude frames and clocks

Use the factory file `/data/matty/m04-dh2-oak-calibration.json`, device
`14442C10116FDAD600`. Keep its provenance and a geometry identifier with outputs.

The raw-enabled indoor set supports the following rotation from raw IMU vectors
into the coordinate frame used by the fused quaternion:

```text
C_FR = [ 0 -1  0 ]
       [-1  0  0 ]
       [ 0  0 -1 ]
```

It was chosen on flat/uphill gravity and checked on single-wheel postures and
full-vector gyro/quaternion derivatives. Implement this as an explicit verified
convention for this hardware/software, not an undocumented global OAK assumption.
Confirm quaternion direction, signs and factory transform directions with unit
tests and recorded data. Normalize quaternions, handle q versus −q, use SLERP;
do not diagnose this mounting from near-singular raw Euler angles.

Use integrated OAK orientation for front-section tilt initially. ESP32 attitude
provides separate rear-section evidence after its mounting/conventions are
verified. Use measured joint angle; never substitute desired steering. The flat
runs support approximately +joint/2 front yaw and −joint/2 rear yaw, but general
3D attitude composition must use rotations, not additive roll/pitch/yaw shortcuts.
Align heading to the run's odometry reference; establish yaw-fusion policy using
gyro/encoder agreement. Do not rely on magnetic heading as an absolute reference.

Raw acceleration norms are about 9.1–9.3 m/s² at rest in this set. Investigate
bias/scale before quantitative acceleration use. Do not double-integrate it into
position/Z or fuse it again as independent attitude information already contained
in the integrated quaternion.

Depth sequence timestamps and camera IMU timestamps have different observed
epochs (`getTimestamp()` versus `getTimestampDevice()`). Define and record their
clock mapping; use per-sample times within batches, not arrival time for all
samples. Assess latency/jitter and bounded interpolation gaps separately. If the
current metadata cannot establish an accurate transform, retain an explicit
approximate host-time mode and measure its fixed-scene projection error.

**Exit:** stated frame/clock transforms, repeatable posture results, raw/fused
angular agreement, and uncertainty/mode labels. No assumed precise sensor latency.

## 2C. Joint-centred articulated motion model

Begin with a verified flat-ground model before extending it to tilted sections.
The state should describe `p_J`, `q_F`, `q_R`, measured articulation and encoder
history. Choose the translation update only after 2A identifies the available
wheel measurements.

- Derive axle velocities and their relationship to joint velocity using measured
  geometry and section orientations. Include rigid-body lever-arm terms: a front
  camera can move during zero-propulsion steering even if the joint were fixed.
- Reconstruct front/rear turning contributions from labelled wheels and compare
  with OAK gyro/attitude, ESP32 yaw and measured joint change. Avoid counting joint
  rotation twice when an absolute section orientation already contains it.
- Evaluate signed encoder-increment integration against reported-speed integration
  on the same data. Preserve actual sample intervals and reset handling.
- Do not force joint translation to zero because desired speed is zero. Conversely,
  do not interpret all wheel travel during steering as forward motion.
- Use a fixed initial alignment between sensor attitudes and the shared odometry
  frame, rather than resetting one section independently during a run.
- Track translational observability. Initial planar mode may hold joint Z at zero
  **as an explicitly reported assumption**. A later wheel/attitude dead-reckoned
  Z requires a stated rolling/contact model and validation; neither two IMUs nor
  a pose3d message provides terrain elevation automatically. Do not promote an
  assumed/weak Z estimate into a claim of accurate full-3D localization.

Parameterize wheel/axle spacing, section reference offsets, IMU mounting and
camera transform. Confirm rear axle position, wheel track/radius, bumper extents
and the housing-reference-to-optics transform; front axle X alone does not specify these.

**Exit:** a small documented model with explicit assumptions, reproducible outputs
and reference-point tests, not a general-purpose sensor-fusion framework.

## 2D. Slip and confidence diagnostics

Compare observed wheel increments/gyro rotation with the articulated model's
predictions. Report per-wheel residuals and inconsistent episodes, with contact
and encoder/power flags. Distinguish wheel-side disagreement, missed packets,
quantization, articulation transients and actual external motion evidence.

A bumper/contact interval with wheel travel is useful evidence of possible slip
or blocked motion; it is not proof of exactly zero chassis displacement. More
importantly, **all four wheels can slip consistently**, so wheel agreement is not
an absolute travel reference. Use fixed-scene depth/visual motion checks or measured
floor displacements for independent evidence. Do not call an encoder residual a
complete slip detector or silently correct travel to an arbitrary value.

First version: publish diagnostics/quality flags. Only introduce rejection or
weighting of suspect increments once negative controls show it improves the pose.
When translation is unreliable, expose the condition to later mapping rather
than presenting an unqualified pose.

## 2E. Reusable shadow-mode node and interfaces

Proposed package: `matty/` in osgar-apps, separated from DH2 mission behavior.
Add it to the existing build package list when implemented. Use OSGAR Node,
quaternion, logging and replay facilities; do not duplicate their lifecycle.

Proposed outputs (OSGAR pose3d = `[[x,y,z],[qx,qy,qz,qw]]`, metres):

- `pose3d`: canonical front orientation at the joint-reference position.
- `rear_pose3d`: rear orientation at the same joint-reference position.
- `camera_pose3d`: derived rectified depth-optical pose, **only if needed**; declare
  its right/down/forward optical axes explicitly. Generic driving consumers must
  use front-body pose3d, not assume optical X is forward.
- `pose_quality`: mode (`planar_assumed_z` / validated wheel-attitude mode), time
  alignment status, source age, encoder validity and slip/model diagnostics.

A mapper can instead consume canonical `pose3d` plus fixed `T_front_depth`;
choose one transform boundary and avoid applying extrinsics twice. Record poses
and acquisition-time metadata sufficient to inspect interpolation at depth time.

Offline audit can recover encoders from incoming `serial.raw` now. For runtime,
prefer publishing the already-decoded per-wheel data through one narrowly scoped
Matty platform interface if the audit demonstrates the need. Prepare that as a
separate reusable driver change for PR review; do not quietly alter legacy
`pose2d`. Avoid maintaining a second long-lived copy of the ESP packet parser in
the mapping application. A temporary offline adapter must be clearly scoped.

Run the new node beside the existing controller, without steering links. Use the
single DH2 config and prefix-specific geometry lookup. Old logs retain their
embedded config; new pose streams are additional analysis outputs. Deterministic
publication comes from recorded inputs/acquisition times, never wall-clock timing.

**Exit:** focused tests, repeatable pose-node replay, explicit degraded-data modes,
and a configuration that acquires/logs pose outputs without influencing motion.

## 2F. Validation and completion gate

Use the existing recordings before asking for additional hardware trials:

| Inputs | Required checks |
| --- | --- |
| Raw posture set `174346`, `174419`, `174456`, `174534` | Front/rear attitude separation, half-joint flat yaw, timestamp/frame handling, encoder patterns during look-around |
| Earlier sets `171019`/`171057`/`171149`, `172750`/`172827`/`172909`/`172947` | Explicit fused-only historical mode; no invented raw samples |
| Box trials `175816`, `175940` | Pose quality through depth stop/contact; retained scene features through translation and articulation; no claim all later contacts were forward retries |
| Event `101546` final approach | Historical ESP-only limitations and pose drift; baseline controller replay preserved |

Synthetic tests: framing fragmentation/escaping/checksum, wheel counter wrap and
reset, signed motion, irregular intervals, zero-speed steering, rigid camera lever
arm, quaternion sign/SLERP, frame compositions, heading wrap and missing bracketing
samples. Tests must challenge observable behavior, not simply mirror formulas.

Fixed-scene check: project selected static surfaces at successive depths using
estimated camera poses. Plot registration residuals through stationary scans and
forward motion; separate nearby moving objects from static features. Report both
median and tail errors, including the provisional rectified-depth intrinsics and
any clock uncertainty. Determine a numerical pass threshold from independent
geometry/timing error estimates **before enabling map decisions**, rather than
tuning it to one successful screenshot.

After offline checks, use a short indoor validation session:

1. Flat, textured fixed scene: neutral and ±30° stationary articulation.
2. Measured straight forward/reverse displacement, using a mark fixed to a named
   body reference point and measurements independent of encoders.
3. Known front/rear support postures, with normal closed lid and a recorded physical
   ESP32 mounting location. Existing posture logs already cover initial checks.
4. Compare estimated joint/front/camera motion and static scene registration. Keep
   the old controller or operator command source; the pose node remains passive.

Stage 2 is complete when the encoder semantics and reference frames are documented,
the shadow provider replays deterministically, front/rear attitudes and camera
lever-arm behavior are verified, independent motion checks quantify translation
error, and any assumed Z/slip limitations remain visible in outputs. Mapping does
not require pretending unobservable quantities are measured.

## First session next time

1. Implement packet/encoder extraction and diagnostics for `174346` and `175940`.
2. Confirm wheel order/sign/units and compare transmitted encoder mean increments
   against the current reported-speed/0.1 s integration.
3. Extend the audit to all raw posture and box cases; write findings before choosing
   the translation model. Proceed to the pose provider after that decision.

No further recordings are required to start this work. This handoff introduces
no runtime or dependency changes.

## Evidence and handoff links

- [Stage 1 procedure and completion](dh2-indoor-stage1.md).
- [Raw capture, frame mapping and validation](dh2-indoor-261102-raw.md).
- [Box contacts and stationary articulation risk](dh2-box-261102.md).
- [Rolling obstacle-map proposal](dh2-local-map-mvp.md).
- Factory calibration: `/data/matty/m04-dh2-oak-calibration.json` (external to Git).
- Existing offline helpers: `dh2.sensor_check`, `dh2.viewer`, `dh2.mapping`.
