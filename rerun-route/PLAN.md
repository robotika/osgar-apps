# Rerun Route Development Plan

This document outlines the roadmap for the `rerun-route` application, moving from simple odometry-based path following to a robust, sensor-fused navigation system.

## Version 0: Odometry-based Following (Current)
*   **Goal:** Basic path following using recorded `pose2d` data.
*   **Mechanism:**
    *   Extract `(x, y)` coordinates from an OSGAR log file.
    *   Use `osgar.followpath.FollowPath` to navigate the resulting route.
*   **Limitations:**
    *   Requires the robot to start exactly at the original starting position.
    *   Susceptible to odometry drift.
    *   No correction from external sensors.

## Version 1: Robust Path Following & Sensor Integration
*   **Goal:** Increase reliability and handle initial position offsets.
*   **Key Features:**
    *   **Path Joining & Offset Handling:**
        *   Implement a "Joining" state to navigate from the current position to the nearest point (or the start) of the recorded route.
        *   Add a configurable threshold to determine whether to navigate back to the start or simply "snap" to the closest part of the route.
    *   **IMU Integration:** Use IMU data (Orientation/Heading) to supplement odometry, providing a more stable heading reference, especially during turns.
    *   **Visual Feature Tracking:**
        *   Incorporate camera data from the OAK-D Pro.
        *   Implement basic visual feature tracking (e.g., using OpenCV) to maintain alignment with the original run's visual environment.
        *   Correct longitudinal and lateral drift using recognized landmarks or features.

## Version 2+: Hardware-Accelerated Visual Navigation
*   **Goal:** Offload processing and improve real-time performance.
*   **Key Features:**
    *   **OAK-D Pro Offloading:** Move feature detection and tracking logic directly onto the OAK-D Pro hardware (DepthAI) to reduce host CPU load and decrease latency.
    *   **Advanced Visual Odometry/SLAM:** Potentially incorporate more sophisticated visual-inertial odometry (VIO) for high-precision tracking in challenging environments.
    *   **Dynamic Obstacle Handling:** Improve the interaction between route following and real-time obstacle avoidance.

## Architecture Considerations
*   **Modularity:** Keep the route extraction logic separate from the control loop to allow for different "planners" or "correctors."
*   **OSGAR Patterns:** Continue following the `Node` and `BusShutdownException` patterns for clean integration and termination.
*   **Replay-ability:** Ensure all new sensor integrations remain testable via `osgar.replay`.
