# Rerun Route TODO List

## Completed Tasks [X]
- [X] Initial application structure (`main.py`, `rerun-route.json`)
- [X] Path extraction from OSGAR logs (odometry-based)
- [X] Automatic visual landmark extraction (ORB descriptors + poses) from reference log
- [X] Robust video stream extraction (in-process `LogReader` with `deserialize`)
- [X] Heading offset estimation using visual landmarks and Homography/RANSAC
- [X] Integration of pose and heading correction into `main.py` control loop
- [X] Emergency STOP termination logic (via `EmergencyStopException`)
- [X] "Standard Development Procedure" documentation (`GEMINI.md`, `DEVELOPMENT.md`)

## Current Goals [ ]
- [ ] Evaluate 2026-04-16 logs to analyze "slightly off" path behavior
- [ ] Improve translation (x, y) estimation from visual landmarks (current estimation is "nearest landmark" only)
- [ ] Implement "Joining" state to navigate from current position to the nearest point on the route

## Future Enhancements [ ]
- [ ] Performance optimization (caching ORB descriptors for reference logs)
- [ ] OAK-D Pro hardware offloading for feature tracking
- [ ] Robustness to more significant lighting changes
- [ ] Robust path resolution relative to `root_path` config (after OSGAR release)
- [ ] IMU integration for stable heading correction during the run

## OSGAR Improvements [ ]
- [ ] Propose/Implement `resolve_path(config, path)` in OSGAR core to handle absolute/relative paths robustly across environments.
