# click2go revival review — 2026-11-12

## Scope and baseline

Reviewed `feature/click2go_v2` at `9f9b4ff`, refreshed origin, and compared
with `origin/master` at `29b8d5b`. The branch is 17 commits behind and four ahead.
Master has not changed click2go since the common ancestor. A read-only
three-way merge preview showed no conflict markers. No merge was performed;
the checkout also contains unrelated untracked application directories.

The original feature was merged as `33aa9e4` (#20) on February 6. The four
February 8 branch commits add frame/command indices, reduce camera acquisition
from 20 to 10 Hz, log the desktop viewer, and add standalone UDP echo examples.
The UDP scripts are not wired into click2go and do not transport video or clicks.

## What was actually tested

Inspected all 33 `*click2go*.log` recordings found directly in `/data/matty`:
32 robot recordings and one dummy recording, January 13 through February 8.
These include several protocol generations; old logs cannot all be replayed
against today's four-field command parser unchanged.

| Recording | Received clicks | Published images | Evidence |
| --- | ---: | ---: | --- |
| `m03-matty-click2go-260125_182612.log` | 23 | 0 | Early raw-coordinate protocol; changing pose and movement outputs |
| `m03-matty-click2go-260126_175127.log` | 19 | 162 | Pose plus click protocol; changing pose and movement outputs |
| `m04-matty-click2go-260207_122654.log` | 18 | 193 | Pre-v2 pose/click protocol; changing pose and movement outputs |
| `m04-matty-click2go-260208_112756.log` | 3 | 21 | Indexed v2 commands; 98 nonzero steering/speed messages, pose remains zero |
| `m04-matty-click2go-260208_113041.log` | 5 | 35 | Indexed v2; pose ends at [552, 115, 1833] |
| `m04-matty-click2go-260208_113433.log` | 46 | 126 | Indexed v2; 466 nonzero movement messages; changing pose |

Pose units are millimetres and hundredths of a degree. Counts establish recorded
inputs/outputs and odometry, not independent ground truth or successful target
arrival. The zero-pose run does not establish whether the robot was physically
blocked, disabled, or had an odometry issue.

Baseline replay of the final recording passed output assertions without force:

```sh
cd click2go
../.venv/bin/python -m osgar.replay \
  /data/matty/m04-matty-click2go-260208_113433.log --module app
```

This used the existing Python 3.10.22 environment. No hardware was contacted.
No click2go unit tests or written physical acceptance report were found in this
branch. `zmq_server.py` is a manual JPEG transport exercise and requires a
`test_image.jpg` file absent from the tracked click2go directory. The dummy
configuration has no image or pose producer, so it cannot exercise the full loop.

## Current behavior and gaps

`Click2Go` is a click-triggered short-step driver, not yet image projection:

- Every command requests forward speed 0.2 m/s in the robot configuration.
  Distance is hard-coded to 0.2 m. Click Y is ignored; X selects +20 degrees,
  straight, or -20 degrees using fixed 1920-pixel thirds.
- Distance is displacement from a periodically reset pose, not accumulated path
  length. A new command does not reset the start pose. Arrival at a clicked
  ground point is not implemented.
- The command carries desktop index, displayed robot frame index, old pose,
  and click coordinates. The controller ignores both indices and the old pose;
  there is no age, duplicate, session, bounds or structure validation.
- `timeout` is stored but never enforced. Emergency stop is only stored and
  not used; the robot config does not link `platform.emergency_stop` to the app.
  Platform/firmware protections are separate from this missing app behavior.
- Movement outputs are updated only on pose input. With frozen pose, the
  command can remain active indefinitely. The local Matty driver resends its
  cached command on ticks, so its serial watchdog is not an application command
  expiry mechanism.
- Image publication uses two exact packet-prefix tests. Pose is the latest
  received pose at publication, without capture-time synchronization. Camera
  FPS is not the transmitted keyframe rate.
- The viewer decodes video via a temporary file. JPEGs are not downscaled,
  although clicks are always multiplied by two; the manual JPEG server therefore
  does not have the same coordinate mapping as the video path.
- Pending clicks are associated with the latest received metadata, potentially
  after reception/decode changed the image. PUB/SUB has no application command
  acknowledgement. The configured TCP endpoints bind all interfaces.

## FGI monitoring work available today

Reviewed `/home/md/git/fgi` branch `feature/online-streaming` at `74db8ab`,
including `5a1dfa1`, `tools/live_monitor.py`, its HTML and tests, and
`doc/LIVE_MONITOR.md`. This review did not modify FGI.

The independent process tails an OSGAR log, serves latest state over loopback
HTTP through SSH, and can automatically follow the newest recording in a
directory. Sessions isolate caches across recordings; the UI labels stale data,
catch-up, errors and disconnects. It has no control bus or command route.

Defaults are telemetry polling up to 2 Hz, an 8 KiB/s shared response-body
budget, and RGB snapshot attempts no faster than every five seconds. Snapshots
decode independent HEVC pictures, retain parameter sets, and bound JPEGs to
640 pixels wide and 16 KiB. These defaults suit observation, but need explicit
freshness handling before using displayed images to request motion.

All eight `tools.test_live_monitor` tests passed on this workstation using FGI's
own environment. Loopback HTTP required execution outside the sandbox. FGI's
documentation also records a 77 MB real-log check producing an 11881-byte JPEG;
robot/SSH/bandwidth-limited validation remains outstanding. The telemetry
allowlist and lidar/geometry display are FGI-specific; Matty steering and
click2go frame identity would need adaptation. Do not transplant its dependency
stack into osgar-apps merely to reuse monitoring code.

## Recommended sequence

1. Integrate current master into the feature branch, retaining these four
   commits. Check incoming `AGENTS.md`, `DEVELOPMENT.md`, uv environment and
   application tests. Account for untracked directories that overlap master
   before changing the checkout. Repeat the passing v2 baseline replay.
2. Bring up read-only Matty monitoring using the FGI sidecar pattern. First
   validate actual robot load, SSH reconnection, HEVC decoding, snapshot age,
   and recording switches. Keep observation independent of robot control.
3. Implement projection offline: calibrated RGB intrinsics/distortion and camera
   pose relative to the robot, intersect the image ray with a ground plane,
   reject horizon/behind-camera/out-of-range clicks. Aligned depth could support
   a later alternative, but the existing enabled depth stream is unused here.
   Preserve source image dimensions, frame identity, capture time and pose;
   transform the target into a persistent odometry frame.
4. Preview the resulting target on recorded images before driving. Add focused
   tests for projection, scaling, frame association and command rejection.
5. Add a separate, explicit command path with accepted/rejected/completed
   feedback, session and frame validation, bounded movement, command expiry,
   stop/abort behavior and tested emergency handling. Begin with short steps
   and a local operator; verify physical stopping separately from log replay.

The first useful milestone is a current-master-compatible camera monitor with
click-to-target preview and no actuator output. That resolves viewing and
geometry before coupling them to motion.
