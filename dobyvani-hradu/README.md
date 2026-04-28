# Dobyvání hradu (Conquer the Castle)

Autonomous mobile robot competition focused on localizing traffic cones within castle ruins.

## Overview
The application uses OSGAR for robot control, OAK-D Pro camera for cone detection, and GPS with geofencing for spatial awareness.

## Key Features
- **Geofenced Navigation**: Search for cones within a predefined boundary.
- **Traffic Cone Localization**: Identify and approach cones using computer vision.
- **Visit Verification**: Wait at each cone and mark its position to ensure all targets are found.

## Setup
This project uses `uv` for dependency management.

```bash
# To run the application (example)
python -m dobyvani-hradu.main dobyvani-hradu.json
```

## Configuration
The behavior is configured via `dobyvani-hradu.json`, including:
- Geofence coordinates
- Camera model blobs
- Speed and timeout parameters
