# Matty camera monitoring and ground preview

The new monitor reads an OSGAR recording in a separate process. It has no OSGAR
bus, serial device or command connection. It serves latest recorded Matty
telemetry and occasional RGB snapshots over loopback HTTP. Browser clicks can
preview flat-ground coordinates when calibration is supplied; they never drive
the robot. The old `click2go.py` controller and its configurations are unchanged.

## Try an existing recording

From the repository root, using the existing Python 3.10 uv environment:

```sh
.venv/bin/python click2go/live_monitor.py \
  /data/matty/m04-matty-click2go-260208_113433.log
```

Open `http://127.0.0.1:8765`. Historical recordings are correctly labeled stale.
This is a latest-state monitor, not time-controlled replay. Click a snapshot to
inspect the original 1920×1080 pixel, even though the transmitted JPEG is smaller.

## Record a new M04 camera sample

The optional `config/camera-monitor.json` records only the camera: no Matty
platform, serial device or motion application is configured. It can be used for
a stationary ground-calibration sample. It does not record robot odometry.
On M04, from the repository root:

```sh
# Uses the normal OSGAR_LOGS directory; retain M04's existing robot prefix setup.
.venv/bin/python -m osgar.record click2go/config/camera-monitor.json \
  --duration 90 --note 'M04 click2go stationary ground sample'
```

This command opens the camera, so stop other applications using that camera
first. Do not run the old motion-enabled click2go config just to test viewing.
For a manually driven sample with pose/joint telemetry, use M04's established
recorder and manual driving procedure instead.

## Monitor over SSH

Leave the recorder in its usual persistent terminal session. From a second
desktop terminal (substitute SSH address, checkout path and log directory):

```sh
ssh -t -L 8765:127.0.0.1:8765 USER@M04 \
  'cd /path/to/osgar-apps && exec nice -n 10 .venv/bin/python click2go/live_monitor.py /path/to/logs --pattern "*camera-monitor*.log"'
```

Open `http://127.0.0.1:8765` on the desktop. When monitoring an existing normal
recorder, adjust or omit the filename pattern. A directory may be empty when
the monitor starts. It selects the newest valid recording by the start timestamp
in its header, not modification time, and clears all cached data when switching
runs. An exact filename pins monitoring to that file. Ctrl+C stops the monitor;
stop the recorder separately. Losing monitoring does not stop robot movement.

Defaults copied from the FGI monitor are an 8 KiB/s response-body budget shared
across browser tabs, telemetry polling up to 2 Hz, and snapshot decode attempts
at least five seconds apart. Snapshot timing also depends on HEVC random-access
pictures. JPEGs are at most 640 pixels wide and 16 KiB. HTTP/SSH overhead is
additional. Insufficient budget returns HTTP 429 instead of queuing updates.
`--rgb-interval 0` disables snapshot decoding. `--port` and
`--kib-per-second` change the loopback port and budget.

The reader scans from the start on attach, skips historical video decoding
while catching up, and retains latest values rather than a network backlog.
It consumes disk I/O and CPU; measure load on M04. Supported camera encoding is
HEVC/H.265 or JPEG; H.264 is not implemented. The displayed image timestamp is
log receipt time, not a calibrated acquisition timestamp. Snapshot identity is
local to this monitor session, not the old app's `remote_count`.

## Optional ground preview

Provide `--calibration /path/to/m04-ground.json`. Without it, pixel inspection
works but metric projection is disabled. Calibration must describe the actual
RGB stream and the camera mounting relative to the chosen robot reference point.
No mounting measurements are inferred from old logs.

The required JSON schema is illustrated below with **synthetic test values,
not M04 calibration**:

```json
{
  "image_size": [1920, 1080],
  "camera_matrix": [[1000, 0, 960], [0, 1000, 540], [0, 0, 1]],
  "distortion_coefficients": [0, 0, 0, 0, 0],
  "camera_to_robot_rotation": [[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
  "camera_position_m": [0.1, 0, 0.5],
  "max_range_m": 5
}
```

`camera_matrix` uses source-image pixels, zero skew and positive focal lengths.
Distortion uses OpenCV's pinhole model (4, 5, 8, 12 or 14 coefficients).
Optical axes are right/down/forward. Robot axes are forward/left/up in metres;
ground is z=0. Rotation maps an optical vector into robot coordinates, and
position is the optical centre in that robot frame. The example rotation is a
level, forward-looking camera. Use measured pitch/roll/yaw for M04. Keep Matty's
joint angle fixed for this initial experiment, or define the frame on the camera's
own rigid robot section. Dynamic articulation compensation is not implemented.

The server undistorts the selected pixel and intersects its ray with the ground.
It rejects sky/horizon, behind-robot and excessive-range targets, nonfinite or
out-of-image pixels, mismatched source dimensions, and outdated snapshot/session
IDs. A selected point is relative to the robot at that image, not current world
odometry. Recorded pose is shown as telemetry; it is not assumed synchronized
with the camera. Historical/stale images may be inspected offline, with their
age kept visible. A new snapshot clears the previous selection.

## Suggested M04 capture

1. Place M04 on level ground with unchanged camera mounting and focus. Record
   camera identity, image resolution, joint position, camera height, and camera
   position/orientation relative to a clearly defined robot reference point.
2. Place visible marks at measured forward and lateral ground coordinates,
   including left/right points and near/far distances (for example 1, 2 and 3 m).
   Keep M04 stationary for 30–60 seconds with the marks in view. Save the actual
   measurements and identify the marks in a note alongside the recording.
3. Export RGB intrinsics/distortion for that device and resolution with the
   existing `rerun-route/get_oak_calibration.py` utility when no recorder owns
   the camera. Its internal stereo extrinsics do not give robot mounting pose.
   `/data/matty/m04-dh2-oak-calibration.json` is an existing candidate export,
   but device identity, resolution/crop and mounting still need verification.
4. View the recording, click the marks with measured calibration loaded, and
   compare forward/lateral errors. Include a horizon and out-of-range click to
   confirm rejection. This step needs no motion controller.
5. In a separate normal manually driven recording, observe snapshot ages and
   resource load; disconnect/reconnect SSH and start a second recording to verify
   stale indicators and cache reset. Robot-side behavior must remain independent
   of the monitor. Five-second snapshots are for inspection, not driving feedback.

## Validation and provenance

```sh
cd click2go
../.venv/bin/python -m unittest -v
cd ..
.venv/bin/ruff check click2go/ground_projection.py click2go/live_monitor.py \
  click2go/test_ground_projection.py click2go/test_live_monitor.py
```

The backend and baseline isolation/directory tests were adapted from FGI's
`tools/live_monitor.py` and `tools/test_live_monitor.py` at `74db8ab`. The local
copy removes FGI lidar/RPM/geometry handling and adds Matty telemetry and optional
frame-bound projection without changing FGI or OSGAR core.

Workstation checks: 13 tests pass, covering growing logs, immutability, latest
values, directory selection/cache reset, HTTP read-only behavior, bounded JPEGs,
frame/session checks and synthetic projection including distortion and tilt.
The 125652085-byte February 8 M04 log was read to EOF, with a 1920×1080 source
decoded to an 8731-byte JPEG; source SHA-256 stayed unchanged. Headless Chromium
loaded that recording, displayed RGB and marked it stale. The unchanged motion
controller's baseline replay passed after master integration. A browser click on
the scaled image mapped to source pixel (960, 1040) and the expected (1.10, 0.00) m
point with synthetic calibration. Hardware, SSH/load
and measured M04 ground projection validation remain to be done.
