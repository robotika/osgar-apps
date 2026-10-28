# DH2 handoff: Dobyvani hradu II / Conquer the Castle

Review date: 2026-10-28. Event: Zviretice, 2026-10-31 (provided by the operator).

## Objective and constraints

Explore the castle area and discover up to four traffic cones ("defenders") using
two Matty robots. One robot searches the left part and the other the right part.
Each has an OAK-D Pro as its primary sensor, low-precision GPS, front and rear
bumpers, the camera IMU, and an ESP32 IMU in the controller for four servos.

Keep mission behavior, perception adaptations, and robot configurations in
osgar-apps. Reuse OSGAR interfaces without putting DH2 policy in the shared core.

**Recommendation:** start a new branch from current `master`, for example
`feature/dh2-2026`. Adapt the depth/cone behavior in `roboorienteering/ro.py` into
a separate DH2 application. Use the old castle branch as design history, not as
a runnable starting point. The operator subsequently created `feature/dh2-2026`;
the initial handoff was committed there as `6d93afd` by Aya Rain.

## Operator additions: robots, recordings and perception

The fleet contains five nominally identical Matty robots, distinguished by their
3D-printed color and unique robot prefix. The DH2 pair is:

| Robot prefix | Color | DH2 assignment |
| --- | --- | --- |
| `m04-` | Pink | Participating; left/right assignment still to be decided |
| `m05-` | Orange | Participating; left/right assignment still to be decided |

The confirmed identity setting is `OSGAR_LOGS_PREFIX`, consistent with
`followme/config/matty-tulak.json` and `followme/follow_person.py`. Use `m04-`
for pink and `m05-` for orange; preserve each robot's identity in recordings and
reports.

Many existing recordings are stored in `/data/matty`. The directory is accessible
on this workstation, but its contents have not been indexed or replayed in this
review. The recordings are unsorted; a searchable index is useful future work,
not a prerequisite to inspect selected logs for DH2. A future index should record
filename, robot prefix, date/duration, application/configuration, software revision
where available, streams, model identity and useful scenario notes. Do not infer
these details solely from filenames.

Preferred perception setup: run the dedicated cone detector using a **superblob**
alongside a general YOLO detector for information about people around the robot.
The operator recalls that YOLO v26 worked and believes the superblob is probably
the latest cone model; the exact artifacts/revisions remain to be confirmed.
Repository candidates are `cones-challenge/coneslayer.rvc2.tar.xz` and
`models/yolo26n.rvc2.tar.xz`. These filenames alone do not establish which model
was used successfully or whether a newer cone model exists elsewhere.

Use the `models` array in `robotem-rovne/config/matty-redroad.json` as a structural
reference for parallel inference, replacing the road model with the selected cone
model. Verify driver support, per-model preprocessing/crops, detection stream
identity, frame rate and latency with both networks plus depth and IMU enabled.
Keep cone observations distinct from person detections; older cone handlers
assume every detection is a cone. Record both outputs for replay. People
awareness is requested; a specific person-triggered motion policy has not yet
been defined.

## Repository snapshot and branch inventory

`git fetch origin --prune` completed successfully before the final inventory.
The checkout is `master` at `feed65d`, equal to `origin/master`; the working tree
was clean before this document. There is one local branch and seven remote
feature branches. Counts below are commits unique to each side of
`master...origin/feature/<name>` (behind / ahead). These are ancestry counts:
squash-merged work can still appear ahead, so they do not measure missing code.

| Remote feature branch | Tip / last commit date | Behind / ahead | Relevance and disposition |
| --- | --- | --- | --- |
| `dobyvani-hradu-2026` | `13ac25c` / 2026-04-28 | 12 / 6 | Mission naming, geofence helper, cone/depth scaffold. State machine is unfinished; do not merge as a working controller. |
| `safe-redroad` | `b759336` / 2026-05-15 | 10 / 26 | Depth-filtered road mask, obstacle slowdown/stop, log analysis and visualization. Select ideas after replay validation; direct depth resizing needs geometric verification. |
| `redroad-model-v2` | `d1cb4b2` / 2026-08-30 | 0 / 5 | New road segmentation blob and day/night configurations, plus larger-mask visualization. Optional terrain aid; no cone search or coverage solution. |
| `robotour-2026` | `65c2c52` / 2026-09-22 | 1 / 22 | OSM routing, Navigator tests, GPS extraction and visualization. Useful for offline track review/coarse guidance; GPS/OSM routing alone cannot solve castle exploration. Adds networkx. |
| `matty-follow-matty` | `45ae0b2` / 2026-06-14 | 9 / 44 | Robot/tag following and Vanjee lidar experiments. AprilTag/Tulak work already exists in master through subsequent development; remaining versions differ. Low priority for independent left/right search. |
| `follow-person-ver1` | `e91ba4c` / 2025-11-30 | 17 / 4 | Additional Qorvo/Pozyx ranging and minimum following distance. Requires hardware outside the stated DH2 set; no immediate mission benefit. |
| `click2go_v2` | `9f9b4ff` / 2026-02-08 | 16 / 4 | Remote-image index/time metadata and UDP examples. Potential operator assistance, not autonomous exploration. Client/server message format changes must travel together. |

No whole feature branch is recommended for immediate merge. Prefer small,
reviewable adaptations and preserve the other applications' behavior.

## Useful pieces already on master

| Source | Reusable behavior | Limitations for DH2 |
| --- | --- | --- |
| `roboorienteering/ro.py`, `roboorienteering/config/matty-gps-way.json` | OAK cone detections plus depth; depth strips form a local clearance scan; steering toward cones; stop/report interval; Matty bumper links and coarse GPS bias. Closest functional reference. | Not a coverage planner. Rightmost detection is selected; GPS waypoints stand in for target identity. Emergency-stop handler and rear-bumper handler are empty. Front collision triggers five seconds of reverse without a rear check. |
| `cones-challenge/main.py`, `cones-challenge/matty-cones-challenge.json` | Dedicated cone model, bbox-to-depth example, steering command units, obstacle stop input. Model assets include both `.blob` and `.rvc2.tar.xz`. | Designed to shuttle around cones, not discover unique defenders. Uses minimum valid depth and asynchronous latest detections/depth; no spatial target registry. |
| `followme/tulak.py`, `followme/config/matty-tulak.json` | Target/road fallback, explicit zero command on emergency stop and bumper contact; front/rear bumper wiring. | A later target callback can command motion again after a bumper stop: use persistent inhibition in DH2. Road following is not area exploration. |
| `robotem-rovne/config/matty-redroad.json` | Matty serial/timer wiring, OAK recording, IMU and multiple-model configuration examples. | `stop_dist` is -1 and there is no obstacle-detector link in this configuration. It is not an obstacle-safe DH2 configuration. Its listed generic detector labels do not include cones. |
| `dtc-systems/geofence.py`, `dtc-systems/test_geofence.py` | Existing boundary helper and tests to study before adding another version. | GPS boundaries are coarse hints given the stated precision. Check coordinate order and distance assumptions. |
| `dtc-systems/depth_analysis.py` | Offline valid-depth counts and percentile analysis. | Fixed camera regions; tune from DH2 logs. |
| `rerun-route/` | Recorded-route following with image alignment; implementation includes ORB/PnP/homography beyond the short README description. | Requires reference logs and calibration. Not an unknown-area exploration solution; assess separately if a taught route is permitted. |
| `bevroad/` | Frame extraction, ground-plane projection, mosaics and alignment diagnostics. | Ground-plane assumptions need checking on uneven castle terrain. Offline tools do not provide online localization automatically. |

The merged commit `f26dd39` ("Dobyvani hradu 2026 (#35)") is a separate effort
from `feature/dobyvani-hradu-2026`. It added Pat/Vanjee lidar road following and
Matty-on-Pat/remote configurations. It is already on master, and does not supply
a standalone OAK-only Matty DH2 mission. The title alone is misleading here.

## Findings in the old castle branch

All paths here refer to `origin/feature/dobyvani-hradu-2026`, not the checkout.

1. **No driving behavior:** `dobyvani-hradu/main.py:run_logic` contains placeholders
   in SEARCHING and APPROACHING. Nothing calls `send_speed_cmd`; there is no
   implemented transition to approaching or visited-cone bookkeeping. README
   feature claims describe intentions rather than completed functionality.
2. **GPS interface mismatch:** `on_nmea_data` calls `startswith` and splits a raw
   NMEA sentence, but its configuration connects `gps.nmea_data`. Other current
   apps consume this stream as a dictionary with `lat` and `lon`. Reconcile with
   the installed OSGAR revision before reuse; a dictionary will fail this handler.
3. **Missing platform tick:** the timer connects only to `app.tick`, whose handler
   is empty. Current Matty examples also connect `timer.tick` to `platform.tick`.
4. **Stop handling is incomplete:** emergency stop only stores a flag which the
   logic never checks. No bumper inputs or depth obstacle-avoidance policy are
   connected. Any implemented motion must first pass a common stop decision.
5. **Fragile cone range:** hard-coded 640x400 geometry and square center-crop
   assumptions; no detection/depth synchronization, label filtering, confidence
   policy, or freshness bounds. An empty bbox makes `mask.max()` fail. Derive
   geometry from actual frames and verify RGB/depth alignment on recorded data.
6. **Wait timer type mismatch:** `self.time - self.state_start_time` is compared
   with a numeric seconds value. Use a duration or explicitly convert to seconds
   when implementing the currently unreachable WAITING state.
7. **Placeholders in configuration:** geofence coordinates are not a surveyed
   Zviretice boundary. Model path and OAK options need checking against the actual
   launch directory, OSGAR driver and DepthAI version. Do not treat the README
   launch command as verified.
8. **Geofence caveats:** nearest geometry is computed in longitude/latitude
   degrees before conversion to meters. The random waypoint fallback is a bbox
   center, which need not lie inside a concave polygon. There are no tests in
   this branch's new application directory.

## Proposed DH2 implementation direction

This section is a proposal, not existing behavior or confirmed competition rules.

- Use one application with explicit robot ID and `search_side: left|right`.
  Define left/right relative to an agreed start pose and heading, plus a sketch
  of the assigned sectors. Start the robots with physical separation. A steering
  preference alone does not guarantee separate coverage; track local progress
  and constrain the intended route/sector with measured uncertainty margins.
- Begin with independent operation and combine defender evidence afterwards.
  Inter-robot communication is not established by the current hardware brief.
  Do not infer a four-cone team total by simply adding each robot's count.
- Use depth for local clearance and target range, local odometry for short-term
  progress, and an evaluated IMU heading source. Log both IMUs, confirm axes,
  mounting transforms and drift; do not average their headings blindly. Camera
  orientation fusion settings differ between existing configurations.
- Treat GPS as coarse context rather than a precise sector divider or unique
  cone identifier. Record estimated cone position from robot pose plus camera
  bearing/range, with timestamp, confidence and image evidence. Track repeated
  observations locally; explicitly retain uncertainty and potential duplicates
  when drift prevents reliable association between robots.
- Separate SEARCH, APPROACH, CONFIRM, LEAVE, RECOVER and STOP states. Determine
  what constitutes discovery from the rules before requiring approach or dwell.
  Handle lost targets and previously confirmed targets without endless loops.
- Route all motion through one decision that checks emergency stop, persistent
  bumper state, fresh sensor data and clearance. Unknown/invalid depth is not
  evidence of free space. Bound recovery attempts and elapsed time; rear bumper
  contact must inhibit reverse. A front camera cannot establish rear clearance.
- Check traversability beyond forward obstacles: ground discontinuities, steps
  and drop-offs need explicit evaluation. Existing depth strips do not establish
  that the ground under the planned trajectory is safe.
- Publish Matty `desired_steering` in existing units: speed in mm/s and steering
  angle in hundredths of a degree, positive left. Do not substitute
  `desired_speed` angular-velocity controllers without checking the interface.

Keep the first implementation small: a dedicated app directory, two explicit
robot configurations, a cone observation component if needed, and focused tests.
Avoid making SLAM, OSM routing, a road model update, or inter-robot coordination
prerequisites for the first field trial.

## Next work and acceptance checks

1. Confirm field/rules: start location and heading; sector sketch and no-go areas;
   defender appearance; discovery/approach/dwell requirements; time limit;
   teleoperation and taught-route rules; expected finish behavior.
2. Select useful existing recordings from `/data/matty`, then capture any missing
   coverage on m04/pink and m05/orange: color, depth, cone and YOLO detections, platform
   pose, GPS, both available IMU streams, front/rear bumpers, emergency stop and
   commands. Record robot ID, software revisions, firmware, camera mounting and
   calibration. Verify actual stream names and payloads against installed OSGAR.
3. Validate the dedicated cone model on representative cones, ground and light.
   Verify crop/alignment, valid depth, distance error, false positives, stale
   messages and multiple simultaneous cones before using ranges for approach.
4. Implement stop arbitration and bounded recovery before exploration. Verify
   that later perception callbacks cannot override an active stop, missing depth
   inhibits driving, and rear contact prevents reversing.
5. Add the minimal search/confirm/leave behavior, with left/right configuration
   and observable progress. Check repeated sightings count once, lost-target
   behavior terminates, and fewer than four defenders does not cause an endless
   approach loop. Trial one robot first, then both with physical separation.
6. Follow `DEVELOPMENT.md` for log-based changes: verify baseline replay, analyze,
   change, inspect normal replay differences, then force replay to examine new
   outputs on the old inputs. Replay cannot prove the new trajectory is safe;
   verify movement and coverage in controlled field tests.

## Review scope and reproducibility

This was a source/history/configuration review of all fetched branch references.
No controller was executed, no model was run, and no robot logs were replayed or
field tests performed. The operator subsequently identified `/data/matty` as an
existing recording source. Hardware behavior and compatibility remain
unverified. External event rules were not researched; event details above come
from the operator.

Useful commands for the next developer:

```bash
git branch -avv
git rev-list --left-right --count master...origin/feature/dobyvani-hradu-2026
git diff --stat master...origin/feature/dobyvani-hradu-2026
git show origin/feature/dobyvani-hradu-2026:dobyvani-hradu/main.py
git show origin/feature/safe-redroad:robotem-rovne/main.py
```

Use branch-qualified `git show` for unmerged files. The branch inventory describes
the original review on `master`; subsequent documentation work is on
`feature/dh2-2026`.
