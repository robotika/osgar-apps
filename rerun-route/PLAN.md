# Rerun Route Development Plan

This document outlines the roadmap for the `rerun-route` application, moving from simple odometry-based path following to a robust, sensor-fused navigation system.

## Version 0: Odometry-based Following
*   **Goal:** Basic path following using recorded `pose2d` data.
*   **Mechanism:**
    *   Extract `(x, y)` coordinates from an OSGAR log file.
    *   Use `osgar.followpath.FollowPath` to navigate the resulting route.
*   **Limitations:**
    *   Requires the robot to start exactly at the original starting position.
    *   Susceptible to odometry drift.
    *   No correction from external sensors.

## Version 1: Robust Path Following & Sensor Integration (Current)
*   **Goal:** Handle initial position offsets and refined translation.
*   **Mechanism:**
    *   **PnP Translation:** Use OAK-D depth data and `solvePnP` to calculate exact $(dx, dy)$ and heading offsets during initial alignment.
    *   **Joining State:** Implement a smooth curve-based joining logic to navigate from the start position to the nearest point on the route.
*   **Status:** Initial implementation complete.

## Version 2: Continuous Visual Tracking & Modularization
*   **Goal:** Maintain alignment over long routes and handle odometry drift in real-time.
*   **Mechanism:**
    *   **Landmark Index Tracking:** Monitor the closest reference landmark based on the current estimated pose.
    *   **Throttled Local Matching:** 
        *   Trigger visual checks every ~1.0m or ~2s.
        *   Search only a small window of landmarks (e.g., current index $\pm 3$) to save CPU and reduce false positives.
    *   **Incremental Pose Filtering:** Use an Alpha-filter or a simplified Kalman Filter to smoothly update `pose_offset` without causing jerky steering commands.
    *   **Modular Architecture:** Split the application into specialized modules:
        *   `AlignmentNode`: Handles ORB matching, PnP solving, and pose estimation.
        *   `RerunApp`: Orchestrates the state machine (Wait, Join, Drive) and path following.
        *   `RouteProvider`: Manages loading and indexing of reference landmarks and paths.
*   **Benefits:** Reduces "snapping" artifacts, improves robustness against local feature changes, and allows for easier debugging of individual components.

## Version 3+: Hardware-Accelerated Visual Navigation
*   **Goal:** Offload processing and improve real-time performance.
*   **Key Features:**
    *   **OAK-D Pro Offloading:** Move feature detection and tracking logic directly onto the OAK-D Pro hardware (DepthAI).
    *   **IMU Integration:** Use IMU data for stable heading reference during the run.
    *   **Advanced VIO/SLAM:** Potentially incorporate more sophisticated visual-inertial odometry for high-precision tracking.

## Architecture Considerations
*   **Modularity:** Keep the route extraction logic separate from the control loop to allow for different "planners" or "correctors."
*   **OSGAR Patterns:** Continue following the `Node` and `BusShutdownException` patterns for clean integration and termination.
*   **Replay-ability:** Ensure all new sensor integrations remain testable via `osgar.replay`.
