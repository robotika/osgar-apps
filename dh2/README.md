# DH2 scan-and-advance exploration

The event recordings and their replay results are summarized in
[zviretice.md](zviretice.md). The tested runtime code is preserved for replay.

## Offline local-map preparation

`python -m dh2.mapping` builds an offline surface-map preview from `oak.depth`,
`platform.pose2d` and measured `platform.joint_angle`. It does not change robot
behavior. Depth points are projected into the run's local odometry frame and
accumulated into 10 cm cells, preserving mean/min/max height, sample count and
the number of contributing frames. Zero depth contributes nothing; unobserved
space stays unknown. There is no free-space ray clearing or traversability claim.

From the repository root, an explicitly approximate preview can be generated:

```bash
uv run --locked python -m dh2.mapping \
  /data/matty/m04-dh2-261030_174455.log \
  --output /tmp/dh2-map-174455 --hfov-deg 90 \
  --pitch-deg 0 --camera-height 0.2
```

Here 90 degrees is a provisional depth HFOV assumption, not measured m04
calibration. The mapper requires either this explicit `--hfov-deg` option or
`--intrinsics FX FY CX CY` for the **rectified depth image at its recorded
resolution**. RGB intrinsics and RGB detection boxes are not used. The repository's
m03 calibration has not been substituted for m04 calibration.

Outputs:

- `map.png`: top-down mean observed surface height and observation counts, with
  the odometry trajectory and any reported cone estimates overlaid. Heights are
  relative to the assumed floor at the pose origin; blue is lower, red higher.
- `surfaces.npz`: sparse cell indices/centers, mean/min/max heights, point counts,
  contributing-frame counts, grid resolution, and the sampled poses/timestamps.
- `metadata.json`: source path, geometry/sampling arguments, projection intrinsics,
  counters, original defender reports and limitations.

Sampling defaults to one frame every 0.5 s and every fourth depth pixel, using
0.15–4 m optical depth and projected heights from -1 to +1.5 m. The plot displays
cells seen in at least three frames; all cells remain in the NPZ. Use `--help`
for range, sampling, time-window and grid options.

Projection accounts for articulation with an **unverified** assumption that front
camera yaw is pose heading plus half the measured joint angle
(`--joint-yaw-scale 0.5`). Camera forward/left offsets default to zero and should
be measured and set with `--camera-forward` / `--camera-left`. Positive
`--pitch-deg` looks downward. These extrinsics, the pose-heading convention, and
the depth intrinsics need verification before interpreting distances/heights as
accurate. Changing pitch between recordings requires changing the argument.

Poses and joint angles are interpolated at host depth-log time; yaw is unwrapped
before interpolation. Gaps over 0.3 s and extrapolation are rejected. A known
capture/pipeline offset can be supplied with `--depth-time-offset`. This does not
use the OAK device capture clock or compensate for body roll/pitch yet.

Initial smoke tests processed `174455` (53 sampled frames, 367,372 projected
points, 1,819 cells) and `180742` (81 frames, 578,178 points, 3,578 cells). The first
preview shows lower observed surfaces consistent with the known drop, but is not
a validated drop detector. The latest controller log replayed normally before
this offline work. Six focused tests cover projection axes, pitch, articulation,
offsets, invalid ranges, temporal interpolation and grid aggregation:

```bash
uv run --locked python -m unittest dh2.test_mapping -v
```

Next: obtain m04 rectified depth intrinsics and mounting offsets, validate a
stationary scan against a wall and level floor, then inspect outdoor runs.
Odometry drift is not corrected; this is not SLAM. Separate runs have separate
origins and are deliberately not merged or registered through low-precision GPS.

## Current robot behavior

The active `config/dh2.json` uses `ConeExplore`, recording RGB, cone detections
and stereo depth. Camera IMU, IR illumination and parallel YOLO remain disabled.
Detections are printed and logged on `app.cones`; completed visits are printed
and logged on `app.defender`. Cone range is still an image-size approximation,
not a measurement from aligned depth.

Matty is a four-wheeled articulated terrain platform with a passive middle joint
and a maximum speed of 0.5 m/s. Zero speed with nonzero steering lets the platform
look around. This controller uses that capability rather than assuming it can
rotate in place like a differential-drive robot.

The search ignores left/right robot assignments for now:

1. Stop and look center, left (+30 degrees), right (-30 degrees), then center.
   Hold each position for 1.5 s; ignore acquisition during the first 0.5 s while
   articulation settles. Scanning starts after the first camera/pose messages.
2. If no cone is acquired, advance 0.5 m at 0.2 m/s and scan again. The first step
   is straight; subsequent steps alternate gentle +20/-20 degree arcs.
3. After three consistent detections with confidence at least 0.6, approach the
   largest cone at 0.15–0.30 m/s, slowing as it grows. Preserve the articulation
   that found it and correct steering incrementally from its image position.
4. Empty detections or 0.5 s without a target stop the approach and start another
   scan. Reacquisition again requires three detections. `target_timeout` handles
   lost targets, not general sensor availability.

When a centered cone reaches 65% of detector image height (`stop_height`), the
robot stops and waits 3 s (`visit_wait`) with the cone still observed. Loss of
the cone cancels the wait; more than 10 cm of robot movement resets the timer.
After the wait, it emits one `app.defender` report and attempts to leave the cone.
This remains an image-size stop, not a calibrated 1 m stopping distance.

Search continues until four locally distinct visits (`required_cones`) are
reported or STOP/OSGAR's external duration ends the run. The active configuration
has no overall travel limit. After the fourth report, speed and steering are zero
and the recording remains available until stopped.

## Recovery and repeat sightings

An obstacle or bumper contact interrupts motion and starts a recovery scan:

1. Look left (+30 degrees), right (-30 degrees), then center while stationary.
   Evaluate depth only after each scan pose has settled.
2. Select a clear direction, preferring greater clearance and alternating the
   tie preference on retries. After a cone visit, prefer a side departure.
3. Reorient while stopped and check depth in the chosen direction again. Advance
   one 0.5 m step with cone attraction suppressed, then resume normal scanning.
   If blocked again, scan again. A stalled departure rescans after 6 s.
4. If every direction is blocked, reverse up to 0.3 m at 0.1 m/s and scan again.
   Rear-bumper contact immediately stops reverse motion and prevents further
   reverse commands while pressed. A 4 s maneuver timeout also ends a stalled
   reverse. Only one reverse attempt is allowed until a forward departure succeeds.
   If still boxed in, remain stationary and keep scanning; do not end the search.

`allow_reverse` is enabled in the shared config with the operator's approval.
Front contact inhibits forward commands; rear contact inhibits backward commands.
Emergency stop still ends the application. Ordinary obstacle/contact recovery
does not permanently set the mission's done flag. No coverage guarantee or
rear depth sensing is provided.

For repeat-sighting suppression, the controller estimates cone position in the
local odometry frame using heading, articulation command, image bearing and
range scaled from the image-size stop (assumed roughly 1 m). Estimated positions
within `repeat_radius` (1 m) of a reported cone are ignored. Cone attraction is
also disabled for the first 0.75 m of travel after a report. Reports contain a
local ID, robot prefix, estimated position, stop pose, observation time and actual
wait duration. This registry is per run and per robot: odometry drift, uncalibrated
articulation/range, and closely spaced cones can cause missed associations or
merged targets. Counts are not deduplicated across robots.

## Depth recording and obstacle stop

`oak.depth` is enabled at 640x400, aligned to the left stereo camera, with
`color_depth_alignment: false`. RGB alignment previously attempted a 2024-pixel
depth width and failed at camera startup. This configuration avoids that path
without modifying OSGAR core. Depth is recorded even when forward motion is
inhibited. It is not mapped into cone detection boxes.

The central depth region (30–70% of width, 30–50% of height) is used for a basic
obstacle stop. The lower portion is excluded to reduce ground/grass interference
with the camera mounted about 20 cm above the ground. This is a region heuristic,
not calibrated ground-plane removal; low obstacles can fall below this region.
`app.clearance` logs the distance in meters of the twentieth nearest positive
finite pixel, the valid fraction and the blocked state. The pixel count is
configurable through `obstacle_min_pixels`.

- Stop forward commands at 0.7 m (`obstacle_stop_dist`).
- Once blocked, require at least 1.0 m (`obstacle_resume_dist`) before resuming.
- Wait for the first depth frame before driving. Once frames arrive, zero depth
  does not itself inhibit motion: open space may have no measurable surface.
- At least 20 measured close pixels are needed to stop, so a sparse close patch
  still blocks but a few speckles do not. There is no valid-percentage requirement.
- A newly blocked depth frame stops current forward motion immediately. Cone
  detections cannot override it. Zero-speed articulation remains available;
  obstacle stops do not require restarting if clearance returns.

Speeds, scan angles and the image-size stop are unchanged. Set
`app.obstacle_stop_dist=0` to collect depth without using it to inhibit motion.
This is a central-view obstacle stop, not an avoidance planner, a swept-body
clearance check or a drop-off detector. The region and thresholds require field
validation with the actual mounting angle, grass and obstacles. Ground or the
cone itself can trigger the stop. Do not interpret the depth distance as cone
range or a calibrated 1 m defender visit.

Set `app.max_speed=0` with OSGAR's `--params` to disable forward travel (scanning
still articulates the platform). For a fully stationary collection run also use
`app.allow_reverse=False`, or select
`dh2.main:ConeDetector` as the application driver for detection-only work.

`terminate_on_stop: true` makes a pressed emergency stop raise
`EmergencyStopException` and end the application, allowing OSGAR to shut down
the recording. The controller default is false, following the existing
`robotem-rovne` convention.

Run on the robot, using its existing environment prefix:

```bash
cd dh2
PYTHONPATH=.. ../.venv/bin/python -m osgar.record config/dh2.json
# Add --duration 30 for a short trial.
```

## First trial and replay

Baseline replay of `/data/matty/m04-dh2-261029_185946.log` matched the recorded
controller outputs. STOP was true at 0.244974 s, but the application continued
sending zeros. The log contains color and empty detection messages, but no depth
frames. The supplied camera output reports RGB-aligned depth width 2024 not
being divisible by 16. Disabling depth removes that pipeline from this test;
it does not repair the alignment setup for future ranging.

Normal replay with the detection-only config failed because the old recording had no
`app.cones` output. Forced replay terminates at the STOP event (0.244974 s).
That verifies early termination, not cone recognition or the new hardware config.

```bash
uv run --locked python -m osgar.replay /data/matty/m04-dh2-261029_185946.log \
  --module app --config dh2/config/dh2.json -F
```

Release STOP before recording; a pressed STOP ends the test. The stationary
detector was validated with subsequent recordings, including cone detections in
`m04-dh2-261029_192807.log` and `m04-dh2-261029_192829.log`. The approach controller
was tested with those inputs offline before the approach trials below.

Baseline replay matched both detection recordings before the approach change.
Normal replay with the approach config reports the new motion-output schema;
forced replay produces forward/left commands after acquisition, at 0.03 m/s for
these already-large cones, and a zero command at each recorded STOP event
(11.108635 s and 8.618373 s respectively). Replay uses stationary observations
and cannot validate how the image-based controller behaves as the robot moves.

## Approach trials and exploration replay (2026-10-30)

Both `m04-dh2-261029_195335.log` and `m04-dh2-261029_195444.log` replayed normally
before changing the controller. The operator reports a stationary cone in the
first trial and moving it into view in the second. The original speed taper
commanded roughly 0.03 m/s near the cone; this motivated the higher speed floor.

Normal replay with `ConeExplore` differs as expected. Forced replay commands
zero-speed scanning, approach at 0.15–0.168 m/s on these observations, and zero
speed/steering at each recorded STOP. Synthetic tests additionally exercise a
complete empty scan, the 0.5 m advance, and resuming scanning. Replayed images
were captured along the original trajectory; the new exploration motion and
articulation still require a robot trial.

## Outdoor recordings and depth change (2026-10-30)

All seven `m04-dh2-261030_*.log` recordings replayed normally before this change;
none contains depth frames. Sampled frames show the earlier recordings still
indoors and the final three outdoors. Outdoor detection-positive message counts:

| Recording suffix | Positive / total | Observation |
| --- | --- | --- |
| `121534` | 1 / 591 | Early RGB frames are strongly overexposed; a sampled cone is at the far right edge, outside the central square detector crop. Lighting alone is not established as the sole cause of missed detections. |
| `121650` | 2 / 623 | Both scores are below the controller's 0.6 acquisition threshold. |
| `121811` | 352 / 382 | Frequent detections and repeated approach acquisition. |

Normal replay with depth-enabled obstacle stopping differs from the original
output schema/behavior as expected. Forced replay of `121534` and `121811`
inhibits all forward commands because those logs have no depth, while retaining
stationary scans and STOP handling. Synthetic tests check obstacle interruption,
invalid/sparse depth, hysteresis and release. Camera startup with this depth
configuration and obstacle behavior need a new robot recording.

The operator noted that large free spaces can return zero depth. The first
depth-enabled trial deliberately kept the conservative valid-percentage gate
while collecting data; the review below replaces that policy.

## Sparse outdoor depth correction (2026-10-30)

Baseline replay matched the new recordings. `150742` is the indoor trial and was
excluded from outdoor tuning. In `151116`, valid depth in the original region
never exceeded 3.2%, so the 20% validity requirement blocked all 188 frames.
In `150950`, measured distances were mostly well above the stop distance, but
169 of 181 frames were blocked. Sparse/zero outdoor depth, rather than close
ground returns, was the main cause of the no-motion behavior in these samples.

The validity gate was removed, and the region was raised as described above.
Normal replay now differs on the clearance output as expected. Forced replay
with the active config produces forward commands for all three outdoor logs
(`150950`, `151024`, `151116`) and still ends with zero speed/steering on STOP.
Depth blocked only 2/181, 2/335 and 0/188 frames respectively. The recordings were
made along the old, largely stationary trajectory; this verifies decisions on
those observations, not the new physical trajectory. Test again on the robot.

## Recovery and multi-cone mission replay (2026-10-30)

Baseline replay matched the five new recordings (`152808`, `152901`, `153023`,
`153108`, `153146`). They expose both the indefinite obstacle stop and the old
permanent image-size stop. Normal replay of the new mission differs because it
adds `app.defender` and changes motion decisions. Forced replay exercises recovery,
bounded reverse and zero commands on STOP in all five logs.

`153023` reports one cone at 30.74 s after a 3.06 s wait, then begins departure
recovery. `153108` reports one at 24.63 s after a 3.02 s wait and chooses a forward
departure. Neither recording produces a duplicate report. Synthetic tests exercise
four separate visits, target loss during the wait, repeat suppression, free-side
selection, fully blocked rescanning, and reverse interruption by distance, timeout
and rear contact. The new departure trajectories and unique-cone association
still need field validation; old images follow the old trajectories.

## Offline checks

From the repository root:

```bash
uv run --locked python -m unittest dh2.test_main -v
uv run --locked ruff check dh2
```

These synthetic tests check persistent stops, unknown depth, lost targets,
side preference, parallel detection handling, empty boxes and repeat counting.
The first robot log was replayed as described above. Follow `DEVELOPMENT.md`
for subsequent replay-based changes.
