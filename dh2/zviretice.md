# Zvířetice — Dobyvani hradu II, 2026-10-31

Post-event review of the recordings available in `/data/matty` on 2026-10-31.
The runtime code and configurations are preserved as tested, for replay. This
document records observations and follow-up topics, not changes to robot behavior.

## Scope and replay

- Available: **15 M04 DH2 runs** and **16 M04 wait-for-start logs** from the event
  date. No M05 event recordings were present in this directory at review time.
- Filenames and log headers use **UTC**; local Czech time on this date was UTC+1.
- All 15 DH2 `app` modules replayed normally, **without `-F` or output differences**,
  using the embedded configuration. Each run's last application steering command
  was zero speed and zero steering following a recorded emergency-stop event.
- Replay was checked with the branch snapshot `5f8ff53`, Python 3.10, and the
  dependencies in `uv.lock` (OSGAR Git revision `3db232792a70d6bbed4a6e31ef5135221c7f882c`).
  This establishes replay consistency; it does not establish a robot checkout hash.
- The full repository test runner, `python -u test.py -v`, passed. No controller,
  camera-driver, model, or configuration adjustments were made for this review.

From the repository root:

```bash
uv sync --locked
uv run --locked python -m osgar.replay \
  /data/matty/m04-dh2-261031_104205.log --module app
```

Use the configuration embedded in the log for baseline replay. Overriding it with
`--config` can discard recorded settings; `-F` deliberately skips output matching.
Raw recordings remain external to Git. Earlier indoor/development recordings are
discussed in [README.md](README.md) rather than repeated here.

## Site and behavior used

The site reference is [zviretice.osm.gz](zviretice.osm.gz): castle outline way
24857326, entry bridge way 331858639. The outline spans approximately 60 × 76 m;
it is context, not a traversability map. No geofence or OSM route was active.

All 15 embedded application configurations use `ConeExplore` with the same
settings: ±30° stationary scans, 0.5 m exploration steps, 0.2 m/s exploration,
0.15–0.30 m/s cone approach, image-height arrival threshold 0.65, and a 3 s visit
wait. Obstacle stop/release are 0.7/1.0 m with 20 close pixels as supporting
evidence. Recovery permits one reverse attempt up to 0.3 m at 0.1 m/s until a
successful forward departure rearms it. Four per-run reports finish the search.

RGB video, cone detections, left-camera-aligned depth, pose, measured articulation,
GPS, bumper states and commands were recorded. Cone distance and repeat-sighting
association are approximate; RGB boxes were not calibrated into stereo depth.
There was no active drop-off detector, global coverage planner, loop closure or
cross-robot target registry. Camera level mounting comes from the operator's
setup notes, not a measured mounting angle stored in each log.

## DH2 runs

All names below have prefix **`m04-dh2-261031_`** and suffix **`.log`**. Duration
includes startup and shutdown. Distance is accumulated **odometry**, not surveyed
travel or coverage; slip and handling can affect it. Detections count messages
containing at least one box, including low-confidence boxes—not unique cones.
Brief interpretations combine recorded state transitions with sampled RGB frames.

| UTC filename time | Duration | Odom. distance | Positive / all detection messages | Reports | Brief analysis |
| --- | ---: | ---: | ---: | ---: | --- |
| `101333` | 1:51.5 | 6.5 m | 0 / 1,040 | 0 | Entrance/bridge context in sampled early video. Scan-and-advance exploration; no cone acquisition. Early image is heavily overexposed. Ended just after returning to a scan. |
| `101546` | 6:43.8 | 22.4 m | 310 / 3,965 | 0 | Nine approach acquisitions, all followed by target loss. Thirteen front-bumper activations and one reverse; repeated contact/recovery dominated the end. Bridge visible in early sample. |
| `102711` | 0:33.5 | 0.8 m | 6 / 262 | 0 | Short obstacle-limited attempt with people in the sampled view. No approach acquisition; three recovery scans. Stopped shortly after a departure began. |
| `102806` | 12:26.2 | 41.4 m | 894 / 7,388 | 0 | Longest run. Thirty-one approach acquisitions/losses, three reverses and one front-bumper activation. Last ~82 s remained in obstacle recovery scanning. No visit wait completed. |
| `104205` | 3:34.8 | 14.6 m | 488 / 2,076 | 1 | Cone visible beside masonry at the completed visit. Report at 50.95 s. After departure, repeated recovery and twelve front-bumper activations; ended at contact. |
| `104642` | 0:51.8 | 2.6 m | 61 / 445 | 0 | Three brief approach acquisitions followed by losses. No blocked clearance frames or bumper activations. Returned to scan-and-advance; no completed arrival. |
| `104739` | 0:03.0 | 0.1 m | 0 / 0 | 0 | STOP already pressed at 0.114 s; application exited immediately. No camera frames. Later STOP release in the file does not indicate a restarted mission. |
| `104838` | 5:13.8 | 20.2 m | 268 / 3,066 | 0 | Approach interrupted by depth obstacle stop around 25.4 s. Ten front-bumper activations and one reverse; repeated recovery without a completed cone visit. |
| `105430` | 1:58.9 | 11.6 m | 436 / 1,116 | 1 | Sustained approach to a visible cone by a stone wall; report at 44.77 s. Recovery, one reverse, then scan-and-advance continued. No bumper activations. |
| `105716` | 2:24.4 | 9.1 m | 245 / 1,372 | 0 | Exploration in the grassy ruins, one approach acquisition. Obstacle recovery returned near the end; stopped during the first reverse attempt. No bumper activations. |
| `105954` | 0:47.3 | 1.3 m | 271 / 400 | 0 | Cone visible near a passage in sampled video, but arrival was not completed. Clearance blocked in 336/400 processed frames. One reverse, then ~25 s of recovery scanning. |
| `110047` | 1:19.5 | 5.1 m | 253 / 722 | 1 | Cone visible on the path beside masonry. Report at 26.95 s after a sustained approach. One reverse, renewed exploration and later obstacle recovery. No bumper activations. |
| `110227` | 2:13.9 | 6.0 m | 296 / 1,266 | 0 | One brief approach acquisition/loss; grass and ruins in sampled view. Eight recovery scans and two reverses, with several short departure attempts. No completed visit. |
| `110504` | 4:00.9 | 20.7 m | 1,243 / 2,337 | 1 | Twenty-six approach acquisitions and twenty-one target-loss transitions before reporting a visible cone at 212.49 s. Departed and resumed exploration. No bumper activations or reverse commands. |
| `110921` | 7:36.8 | 42.3 m | 943 / 4,495 | 1 | Visible cone reported at 97.76 s; departure and several minutes of exploration followed without a second report. One front-bumper activation; no reverse. Largest odometric path in this set. |

## Completed report events

Sampled RGB frames shortly before each report show a traffic cone. These are
**five report events across five runs**, not proof of five distinct defenders:
each process resets its local IDs and association registry. No run reached four
reports, and these records alone are not an official game score.

| Recording time | Report offset | Stationary wait recorded | Per-run ID |
| --- | ---: | ---: | ---: |
| `104205` | 50.947 s | 3.018 s | 1 |
| `105430` | 44.774 s | 3.094 s | 1 |
| `110047` | 26.952 s | 3.021 s | 1 |
| `110504` | 212.491 s | 3.090 s | 1 |
| `110921` | 97.758 s | 3.013 s | 1 |

The 15 runs total approximately **51 min 40 s** of recording and **204 m of
accumulated odometry**. There were 37 recorded front-bumper activations and no
rear-bumper activations in the DH2 event runs. These are sensor events; do not
infer which were operator interventions without field notes. The absence of
rear activations does not test the rear-contact interruption rule on site.

## Wait-for-start recordings

All names here have prefix **`m04-matty-wait-for-start-261031_`**, suffix `.log`.
They use `osgar.terminator:Terminator`, not the DH2 mission controller. The first
15 finish on STOP release; their baseline replay prints the terminator's
`EmergencyStopException` thread traceback. This is the release-triggered
termination path, not a DH2 output mismatch. The last file reaches end-of-log
with STOP still pressed. None is an additional exploration attempt.

| UTC filename time | Duration | Brief analysis |
| --- | ---: | --- |
| `101152` | 1:40.1 | STOP pressed, then released before `101333`. |
| `101527` | 0:18.3 | STOP released before `101546`. |
| `102232` | 4:38.1 | Standby interval, then release before `102711`. |
| `102747` | 0:17.7 | STOP released before `102806`. |
| `104034` | 1:29.3 | STOP released before `104205`. |
| `104542` | 0:59.3 | STOP released before `104642`. |
| `104736` | 0:01.7 | Brief release before the already-stopped `104739` attempt. |
| `104747` | 0:50.4 | Starts released; requires a subsequent pressed→released cycle, observed at 41.7→50.4 s, before `104838`. |
| `105354` | 0:34.7 | STOP released before `105430`. |
| `105631` | 0:43.9 | STOP released before `105716`. |
| `105943` | 0:10.1 | STOP released before `105954`. |
| `110044` | 0:02.1 | Brief release before `110047`. |
| `110208` | 0:17.1 | STOP released before `110227`. |
| `110443` | 0:19.7 | STOP released before `110504`. |
| `110907` | 0:13.3 | STOP released before `110921`. |
| `111700` | 22:00.4 | Post-run standby; STOP remains pressed throughout the available recording. |

## Follow-up topics, kept separate from this replay-preserving merge

- Repeated acquisition/loss and recovery loops consumed substantial time. Distinguish
  detector loss, scan/crop effects and association suppression using replay plus video.
- Several sampled startup images are strongly overexposed; report-time samples are
  better exposed. This observation does not establish that exposure explains every
  missed detection or characterize the entire duration of each run.
- Front-bumper activations and short blocked departures show that a central depth
  region is not a full-body passage/traversability check. Terrain capability, gate
  thresholds and drop protection need separate treatment rather than a global
  distance reduction alone.
- Keep image-height arrival and approximate target positions distinguishable from
  measured metric cone range. No calibrated two-stage depth approach was deployed.
- Log channel 0 includes bus-delay diagnostics (some over 1 s). These were preserved
  and do not prevent replay matching; timing and causes require separate analysis.
- Cross-run target identity, coverage and the official result need operator/field
  notes. The offline surface mapper is an inspection tool, not the navigation map
  used during the event.
