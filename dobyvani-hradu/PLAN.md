# Plan: Dobyvání hradu (Conquer the Castle)

## Goal
The goal is to autonomously navigate a mobile robot to localize traffic cones, approach them, wait for a few seconds, mark them as visited (based on GPS), and continue searching for more cones within a defined area (castle ruins) potentially restricted by a geofence.

## Research Phase
- [ ] Analyze `roboorienteering/ro.py` for cone detection and OAK-D Pro camera integration.
- [ ] Analyze `dtc-systems/geofence.py` for geofencing logic and how to integrate it with navigation.
- [ ] Review `cones-challenge/main.py` for cone approaching and "wait" logic.
- [ ] Identify necessary configuration parameters (cone model path, geofence coordinates, wait time, etc.).

## Strategy
1. **Localization & Navigation**: Use GPS and IMU (via OSGAR) for basic movement.
2. **Geofencing**: Implement a "random walk" or "search pattern" within the geofence until a cone is detected.
3. **Cone Detection**: Use the OAK-D Pro model to identify cones and estimate their distance/bearing.
4. **Approach & Wait**: Once a cone is detected, switch to a "targeting" mode to approach it. Wait for X seconds upon arrival.
5. **State Management**: Keep track of visited cones by their GPS coordinates to avoid re-visiting.

## Implementation (Execution)
1. **Project Setup**:
   - [ ] Create `main.py` with basic OSGAR Node structure.
   - [ ] Add `dobyvani-hradu.json` configuration file.
2. **Core Logic**:
   - [ ] Implement Geofence integration.
   - [ ] Implement Cone detection and distance estimation.
   - [ ] Implement State Machine (SEARCHING -> APPROACHING -> WAITING -> SEARCHING).
3. **Validation**:
   - [ ] Unit tests for coordinate tracking.
   - [ ] Simulation/Replay verification with existing logs if possible.
   - [ ] Field testing.

## Dependencies
- `osgar`
- `numpy`
- `shapely` (for geofencing)
- `opencv-python` (for OAK-D processing)
