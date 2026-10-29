# DH2 cone-approach image collection

The active `config/dh2.json` uses `ConeApproach` to collect RGB video while slowly
approaching a cone. Depth, camera IMU, IR illumination and parallel YOLO remain
disabled. Detections are printed and logged on `app.cones`; no metric range
estimation or defender counting is performed.

After three consistent detections with confidence at least 0.6, the robot steers
toward the largest cone's image center. Speed is at most 0.1 m/s and decreases
as the cone grows. Empty detections or 0.5 s without a detection stop motion;
reacquisition requires three detections again. `target_timeout` handles target
loss, rather than general sensor availability.

The robot stops and waits for restart when the cone reaches 65% of detector image
height (`stop_height`), it travels 2 m (`max_travel`), or either bumper is pressed.
These are image-collection limits, not a calibrated 1 m stopping distance. No
obstacle avoidance is available with depth disabled. It does not search or
reverse. Use a clear approach path for this controlled collection test.

Set `app.max_speed=0` with OSGAR's `--params` for a stationary run, or select
`dh2.main:ConeDetector` as the application driver for detection-only work.

`terminate_on_stop: true` makes a pressed emergency stop raise
`EmergencyStopException` and end the application, allowing OSGAR to shut down
the recording. The controller default is false, following the existing
`robotem-rovne` convention.

Run on the robot, using its existing environment prefix:

```bash
cd dh2
PYTHONPATH=.. ../.venv/bin/python -m osgar.record config/dh2.json --duration 30
```

The original config is preserved unchanged in `config/dh2-backup.json`. It uses
the roaming `Castle` controller described below. Its camera alignment setup
failed on the first robot trial; it is a backup, not a validated launch config.

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
is tested with those inputs offline; its actual motion remains to be validated.

Baseline replay matched both detection recordings before the approach change.
Normal replay with the approach config reports the new motion-output schema;
forced replay produces forward/left commands after acquisition, at 0.03 m/s for
these already-large cones, and a zero command at each recorded STOP event
(11.108635 s and 8.618373 s respectively). Replay uses stationary observations
and cannot validate how the image-based controller behaves as the robot moves.

## Roaming ver0 backup

This is the initial controller, not a field-validated release. The development
environment is described in the repository README.

The search strategy is reactive roaming with a preferred turn direction:
drive forward while the center depth corridor is clear; as it narrows, prefer
a clear turn toward the assigned side, otherwise the other side. Approach an
unvisited cone when its range and bearing are available. Stop within 1 m and
require two seconds of stable observations and less than 10 cm of local robot
movement before reporting a defender. Then attempt a short forward turn away.
This does not guarantee coverage or keep the robot within a surveyed sector.

The backup configuration, `config/dh2-backup.json`, selects the search side using
`OSGAR_LOGS_PREFIX`, following the prefix-keyed settings in `dtc-systems`.
Provisional overrides are `m04-search_side: left` and `m05-search_side: right`;
other prefixes use `search_side`. The prefix comes from the robot environment
and is automatically included in log filenames. Both robots use the
cone superblob and YOLO26 concurrently; person detections are logged as
information, without an additional person-specific motion rule. Their common
driver detection stream is filtered by label. Cone observations expire after
0.7 s (`target_timeout`) so a lost target is no longer approached.

OSGAR handles device initialization; the controller waits for its first pose and
depth messages without maintaining a separate sensor-availability watchdog.
The robot stops on insufficient valid depth,
bumper contact, emergency stop or blocked recovery. Contact, emergency stop and
blockage latch until process restart. OSGAR's `--duration` option controls the
recording runtime; the application has no separate runtime limit.
There is no automatic reverse. GPS and IMUs are recorded by the driver modules;
the draft does not fuse them or use GPS for navigation.

## Validation still required

`geometry_verified` defaults to false in the backup configuration. This disables cone
range/approach/counting while leaving depth roaming enabled. Before enabling it,
check that the square detector crop maps to RGB-aligned depth as assumed in
`Castle.target`, measure camera FOV and confirm the installed driver/runtime
supports alignment and both models. Model paths are relative to `dh2/`.

Local cone positions are rough odometry-based estimates. The 2 m repeat-sighting
radius can merge nearby defenders or fail under drift. Reports are per robot;
they are not a deduplicated team score. Recorded video can provide supporting
evidence, but reports do not yet store frame references. The fixed depth strip
is not a ground/drop-off detector.

Once geometry and field behavior have been checked, run from this directory
with the repository on Python's import path. The robot's launch environment
supplies `OSGAR_LOGS_PREFIX`; the shared configuration declares it as null so
OSGAR passes the environment value to the controller.

```bash
cd dh2
PYTHONPATH=.. ../.venv/bin/python -m osgar.record config/dh2-backup.json
```

The config deliberately retains `geometry_verified: false`; change it only after
the geometry check. Use a controlled area with an operator for the first trial.

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
