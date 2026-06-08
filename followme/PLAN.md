# Robot-Following-Robot Development Plan

This document outlines the roadmap and technical strategy for implementing a robust robot-following-robot functionality for the Matty robot using only its built-in OAK-D Pro camera (without LIDAR).

## 1. Requirements & Operational Constraints
*   **Platform:** Matty robot (using the standard `osgar.platforms.matty:Matty` platform).
*   **Sensor Payload:** OAK-D Pro camera only (active IR stereo depth, 1080p color, built-in IMU, on-board YOLOv8 inference). No LIDAR.
*   **Maximal Speed:** 0.5 m/s.
*   **Target Follow Distance:** 1.0 meter (configurable).
*   **Safety Requirements:**
    *   Immediate deceleration/stop when the leader stops or gets closer than 1.0 meter.
    *   Stop immediately if the target is lost or tracking confidence drops below a threshold.
    *   Integrate with front/rear bumpers and physical/software emergency stops.
    *   Visual status communication using onboard LEDs.

---

## 2. Technical Options for Target Tracking
To track the leader robot without a LIDAR, we have three main visual/spatial options using the OAK-D Pro camera:

### Option A: Pure Depth-Based Tracking (Spatial Centroid)
*   **Mechanism:**
    1.  Extract a horizontal slice of the depth map (e.g., around the camera's horizon) from the `oak.depth` stream.
    2.  Filter out noise (invalid/zero values) and find the nearest contiguous cluster/obstacle in a forward-facing Region of Interest (ROI).
    3.  Calculate the centroid of this cluster to get its precise distance (in meters) and angular offset relative to the camera's center.
*   **Pros:**
    *   **Extremely robust day/night:** OAK-D Pro uses active infrared illumination, meaning depth works perfectly in complete darkness (with `is_color: false` configuration).
    *   **High precision:** Provides raw physical distance (m) and horizontal angle without relying on neural network bounding box accuracy.
    *   **Zero training latency:** No custom model training required.
*   **Cons:**
    *   **Distraction risk:** The follower might track and follow any other obstacle (e.g., a tree, wall, or a person crossing between the robots) if it gets closer than the leader.

### Option B: YOLO Bounding Box + Depth Fusion (Semantic Tracking)
*   **Mechanism:**
    1.  Run the standard `yolov8n_coco` model on the OAK-D Pro to detect a proxy class mounted on the leader's rear (e.g., a "backpack", "suitcase", or "sports ball").
    2.  Use the 2D bounding box from `oak.detections` to segment the target.
    3.  Extract depth values only from the pixels within the bounding box to calculate distance and horizontal angle.
*   **Pros:**
    *   **High selectivity:** The follower will only follow the designated target, ignoring walls, trees, and other people.
    *   **Reusable code:** Can extend the existing YOLO-based `FollowPerson` architecture.
*   **Cons:**
    *   **Target attachment needed:** Requires a physical visual proxy (e.g., a backpack) to be mounted on the lead robot.
    *   **Intermittent tracking:** NN detections can drop out momentarily due to extreme angles, motion blur, or lighting changes.
    *   **Night limitations:** In night mode with `is_color: false` (monochrome mono cameras), YOLO models trained on RGB COCO struggle to detect objects reliably.

### Option C: LED Color / Visual Beacon Tracking
*   **Mechanism:**
    1.  The leader robot displays a distinct LED pattern or color (e.g., bright orange/pink) on its rear.
    2.  The follower uses color thresholding on the RGB stream to track the LEDs' centroid and queries depth at those coordinates.
*   **Pros:**
    *   Simple, lightweight visual tracking.
*   **Cons:**
    *   Highly sensitive to daylight, sun glare, and shadows.
    *   Incompatible with night configurations where `is_color` is disabled for optimal IR stereo depth performance.

---

## 3. Recommended Roadmap (Phased Implementation)

We propose a **hybrid architecture** that combines the robustness of **Depth-Based Tracking** with the semantic selectivity of **YOLO Guidance**.

### Phase 1: Robust Distance/Steering Controllers (Foundation)
*   **Objective:** Implement a dedicated follow application (`FollowRobot` or a configurable version of `FollowPerson`) with a closed-loop speed and steering controller.
*   **Speed Controller:** A proportional-integral (PI) controller based on the distance error:
    $$e_{dist} = \text{measured\_distance} - \text{target\_distance}$$
    $$\text{speed} = K_p \cdot e_{dist} + K_i \cdot \int e_{dist} \, dt$$
    Clamped strictly between `0` and `0.5` m/s (no backing up).
*   **Steering Controller:** A proportional controller based on the angular offset:
    $$\text{steering\_angle} = K_{p\_steer} \cdot \theta_{error}$$

### Phase 2: Dual-Mode Tracking State Machine
*   **Mode 1: YOLO-Guided Depth Tracking (Primary)**
    *   Look for a designated class (e.g., `"person"`, or `"backpack"` acting as a proxy).
    *   When detected, compute distance and angle from the depth map inside the bounding box.
*   **Mode 2: Spatial Centroid Tracking (Secondary / Fallback)**
    *   If the YOLO target is lost for less than $T_{timeout}$ seconds, maintain tracking by targeting the nearest spatial cluster within the last known bounding box region.
    *   If no target is detected in either mode, bring the robot to a controlled stop.

### Phase 3: Configuration & Day/Night Integration
*   Create a new config file `followme/config/matty-follow-robot.json` containing:
    *   Standard Matty driver links.
    *   `app` configuration with tunable controller gains (`Kp`, `Ki`), `target_distance = 1.0`, and `max_speed = 0.5`.
    *   OAK-D Pro depth and detection mapping configurations.

---

## 4. Next Steps for Implementation

1.  **Review existing logs:** Examine existing `followme` and `dtc-systems` logs to see standard OAK-D depth outputs and check bounding box size/distances.
2.  **Code Implementation:** Create `followme/follow_robot.py` (or refactor `follow_person.py` to be a generic target follower supporting both `"person"` and `"robot"` modes via depth filtering).
3.  **Local Replay & Testing:** Follow the standard development mandate:
    *   Test depth and steering logic locally using `osgar.replay` on existing logs.
    *   Validate the closed-loop controller's response to changing distances.
