# DH2 ver0 draft

This is the initial controller, not a field-validated release. The development
environment is described in the repository README.

The search strategy is reactive roaming with a preferred turn direction:
drive forward while the center depth corridor is clear; as it narrows, prefer
a clear turn toward the assigned side, otherwise the other side. Approach an
unvisited cone when its range and bearing are available. Stop within 1 m and
require two seconds of stable observations and less than 10 cm of local robot
movement before reporting a defender. Then attempt a short forward turn away.
This does not guarantee coverage or keep the robot within a surveyed sector.

One shared configuration, `config/dh2.json`, selects the search side using
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

`geometry_verified` defaults to false in the shared configuration. This disables cone
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
PYTHONPATH=.. ../.venv/bin/python -m osgar.record config/dh2.json
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
No baseline robot log has been replayed for this new application yet. Follow
`DEVELOPMENT.md` when selecting a log for replay-based changes.
