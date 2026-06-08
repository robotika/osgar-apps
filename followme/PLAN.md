# Streamlined Robot-Following-Robot Plan (Depth-Only)

This document outlines the simplified, high-priority plan to implement a robust robot-following-robot functionality for the Matty robot using **only OAK-D Pro depth data (Option A)**. 

## 1. Context & Objectives
*   **Target Environment:** Day-only, potential bright sunlight (active IR depth is still reliable, but high ambient light requires robust depth parsing).
*   **Maximal Speed:** 0.5 m/s.
*   **Expected Follow Distance:** 1.0 meter (configurable).
*   **Constraints:**
    *   **Time-limited delivery:** Must be fast to implement and calibrate.
    *   **DepthAI Migration:** We are moving to DepthAI v3 (3.6.1) where YOLO integration is changing. Relying on depth-only bypasses any YOLO dependency, ensuring compatibility and stability during the platform upgrade.
    *   **Data Collection:** This depth-only delivery will act as a baseline and a tool to record logs/videos for future neural network-based tracking (Option B).

---

## 2. Technical Approach: Pure Depth-Based Tracking
Without LIDAR or YOLO, we rely entirely on the spatial depth map (`oak.depth`) to segment the leader and calculate steering and speed commands.

### Depth Map Parsing & Centroid Calculation
1.  **Horizontal Slice Extraction:**
    *   Extract a horizontal slice around the camera's horizon (e.g., $Y_{\text{center}} \pm 30$ pixels) from the 640x400 depth image.
2.  **Region of Interest (ROI) & Filtering:**
    *   Define a horizontal window (e.g., center 320 pixels) to ignore peripheral clutter.
    *   Filter out invalid pixels (depth = 0, representing out-of-range or shadowed pixels).
3.  **Target Segmentation:**
    *   Identify the closest contiguous "obstacle cluster" in the forward-facing cone.
    *   Extract a robust distance estimation using a percentile (e.g., 5th or 10th percentile) to represent the rear-most physical boundary of the leading robot.
    *   Calculate the horizontal center (offset) of this obstacle cluster to determine its angular deviation ($\theta_{\text{err}}$).

---

## 3. Control Loop Architecture

We will implement a clean, closed-loop controller in a new node or a specialized subclass.

### Speed Controller
A proportional controller based on the distance error from the target $1.0\text{ m}$:
$$e_{\text{dist}} = \text{measured\_distance} - \text{target\_distance}$$
$$\text{speed} = K_p \cdot e_{\text{dist}}$$
*   **Clamping:** Speed is strictly clamped between `0` and `0.5` m/s.
*   **No backing up:** If $e_{\text{dist}} < 0$ (leader is closer than 1 meter), the speed command is strictly `0` (stop).
*   **Safety Timeout:** If the target is lost or distance jumps to an unrealistic range (e.g., $> 3.0$ meters), speed is immediately set to `0`.

### Steering Controller
A proportional controller that steers towards the tracked centroid:
$$\text{steering\_angle} = K_{p\text{\_steer}} \cdot \theta_{\text{err}}$$
*   If the target is lost, steering is held at `0` or slowly decayed.

---

## 4. Phased Roadmap for Fast Delivery

### Phase 1: Code Implementation & Verification
*   Create `followme/follow_robot.py` (inheriting from `Node`).
*   Extract depth slice logic and implement the P-controllers for speed and steering.
*   Add configuration parameters in the `__init__` constructor:
    *   `max_speed` (default: 0.5)
    *   `target_distance` (default: 1.0)
    *   `Kp_distance` (default: 0.5)
    *   `Kp_steering` (default: 2.0)
    *   `horizon` (default: 200)

### Phase 2: Configuration Setup
*   Create a clean configuration file `followme/config/matty-follow-robot.json` that runs the new `follow_robot:FollowRobot` module, connects it to the Matty platform and the standard OAK depth output, but disables YOLO inference to save CPU/GPU cycles.

### Phase 3: Local Replay & Tuning
*   Test the implementation against existing log files using `osgar.replay` to ensure:
    *   Depth frames are parsed without exceptions.
    *   The node successfully publishes `desired_steering` with correct speed and steering outputs.
