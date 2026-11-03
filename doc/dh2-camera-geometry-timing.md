# Stage 2: camera geometry and clock audit

Offline findings and capture preparation, 2026-11-03. The six raw-enabled M04
posture/box recordings confirm the candidate disparity-to-depth conversion scale.
They cannot recover the missing IMU-device/depth-host clock relationship or
verify rectified principal point from the retained data alone.

## Depth conversion

`dh2.camera_audit` samples every eighth pixel in each depth frame, retaining
0.3–3 m values. Factory left-to-right translation has norm **75.1572 mm**;
resizing the physical-left calibration to 640×400 gives `fx=400.1382` pixels.
Their product is **30.073274 pixel-metres**.

For each sampled depth `Z`, evaluate candidate integer disparity
`d=round(fx*baseline/Z)` and compare `Z` with `fx*baseline/d`.
All **3,696,133 sampled valid depths across six recordings** lie within 1 mm of
these integer-disparity levels. This uses the factory values directly, without
fitting a scale. It supports millimetre depth units and explains the coarse
distant-depth plateaus found in the [static-patch analysis](dh2-static-patches.md).

This does **not** verify `cx/cy`, rectification/cropping, distortion or optical
mounting. Depth values alone do not contain enough evidence to determine those
projection parameters. The factory capture's SDK version is not proof of the
SDK version used to make each recording.

## Recorded timing

The reviewed OSGAR v3 source uses `getTimestamp()` for `oak.depth_seq` and
`getTimestampDevice()` for samples in `oak.orientation_list`. Installed DepthAI
API descriptions identify these as SDK-host-clock and device-clock timestamps,
respectively. OSGAR log stamps use UTC elapsed time. Old logs do not record the
SDK-clock/UTC anchor or paired host/device timestamps for either sensor.

For each stream, subtract its acquisition timestamp from its log receipt time,
then subtract the smallest such difference. This reports **relative receipt
variation**, including buffering and clock effects; it does not estimate absolute
latency or synchronize different clock origins.

| Run | Depth metadata excess P90 / max (ms) | Latest IMU sample excess P90 / max (ms) | Median metadata-to-depth-payload log gap (ms) |
| --- | ---: | ---: | ---: |
| `174346` | 9.3 / 96.1 | 19.8 / 25.4 | 19.1 |
| `174419` | 10.1 / 95.6 | 19.7 / 25.9 | 18.4 |
| `174456` | 9.6 / 85.9 | 20.1 / 27.4 | 18.5 |
| `174534` | 9.3 / 85.9 | 18.9 / 25.0 | 17.9 |
| `175816` | 9.7 / 86.7 | 23.8 / 33.8 | 18.4 |
| `175940` | 10.6 / 86.1 | 23.2 / 32.1 | 18.1 |

All depth metadata/payload messages pair in driver publication order. Payloads
contain no sequence identifier, so this is an ordering check. The metadata is
published before obtaining/serializing/logging the array; that visible gap is
not the exposure-to-host latency. The unusually old first depth frame accounts
for each depth stream's maximum excess. No new timing offset is installed.

## Configuration cleanup

The reviewed v3 driver reads **`depth_profile`**, and never reads `stereo_mode`.
The historical DH2 configs contain `stereo_mode: HIGH_ACCURACY`; it therefore
does not establish that this preset ran. The unused key is removed from the
shared current config. No replacement preset is guessed: the SDK default plus
the explicit median/disparity/left-right overrides remain what the reviewed
v3 driver applies. The repository lockfile requires DepthAI v3.

## Reproduce the audit

```bash
uv run --locked python -m dh2.camera_audit \
  /data/matty/m04-dh2-261102_174346.log \
  /data/matty/m04-dh2-261102_175940.log \
  --calibration /data/matty/m04-dh2-oak-calibration.json \
  --output /tmp/dh2-camera-audit.json
```

[Full six-recording results](dh2-camera-audit.json) retain source configuration,
calibration hash, common depth levels and timing distributions.

## Next robot capture

With the mission stopped so the camera is free, run from the repository root:

```bash
uv run --locked python -m dh2.capture_depth_metadata \
  --output /data/matty/dh2-depth-metadata.json
```

This opens only the camera and builds stereo plus an orientation IMU pipeline;
it does not instantiate the platform or mission. It reads the shared config's
stereo overrides, uses passive illumination, and collects 20 depth frames by
default. It saves:

- Factory calibration and SDK version.
- Depth and rectified-left frame transformation metadata, including source and
  output intrinsics, distortion coefficients and same-sequence alignment claims.
- Both SDK-host and device timestamps for depth and individual IMU samples.
- SDK-clock/UTC anchors bracketed by UTC reads, and sampled depth-value counts.

If the SDK reports no valid frame transformation, intrinsics remain absent;
factory intrinsics are not silently substituted. A valid SDK transformation is
still a reported projection requiring physical verification. Missing IMU data
or a capture timeout raises an error.

For a separate stationary comparison of depth quantization, repeat with
`--subpixel --output /data/matty/dh2-depth-metadata-subpixel.json`. This overrides
only the capture pipeline. Compare a fixed scene; no accuracy improvement or
runtime compatibility is assumed before reviewing the result.

**Clock offsets belong to this boot/session.** A standalone capture does not
calibrate later mission logs after reopening/rebooting the device. These results
will identify which metadata should be recorded alongside depth/IMU during
future missions. This camera-only pipeline also omits mission neural-network
and video-encoder load, so its delivery timing cannot stand in for mission latency.

## Validation and remaining boundary

Seven added tests cover clock-origin ambiguity/reset rejection, known quantized
depth and incorrect conversion scale, calibration units/resizing, paired timestamp
preservation, unavailable/invalid SDK transforms and configuration overrides.
All 87 DH2/Matty tests and focused lint pass. A synthetic frame using the installed
SDK verifies the metadata APIs and JSON output without a camera; normal box pose
replay still matches.

The new hardware pipeline has **not** been run on M04. Next review that capture,
then perform the independently measured straight forward/reverse test in the
[stage 2 plan](dh2-stage2-plan.md). Mapping decisions still need a measured error
budget; neither these offline checks nor reported frame intrinsics establish it.
