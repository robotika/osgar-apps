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

---

## 5. Real-World Robotics & Safety Considerations

To ensure maximum field robustness under practical constraints, the design accounts for several real-world factors:

### A. Articulated Steering & FOV Gating
*   **Mechanism:** Matty features a center pivot joint with the OAK-D Pro camera mounted on the front section. As the robot steers, the front body—and consequently the camera—naturally rotates toward the target. This behaves like a mechanical panning mechanism, keeping the leader centered.
*   **Safety Gating:** If the leader turns extremely sharply and slips out of the horizontal $69^\circ$ FOV, the **target lock memory** retains the last-known horizontal offset for up to $T_{\text{timeout}}$ frames. This allows the follower to complete its turn and re-acquire the target, rather than executing a jarring emergency stop.

### B. Ground & Slope Intrusion (Horizon Pitch)
*   **The Issue:** When traversing uneven terrain or entering a downward slope, pitch variations can shift the ground surface into the horizontal depth slice (`horizon`). This registers as a false-positive close-range obstacle, causing sudden halts.
*   **Mitigation:** 
    *   A highly focused vertical band (`depth_height = 60` pixels) is used to minimize the inclusion of ground planes.
    *   Future versions will support **Active Horizon Adjustments** by reading pitch data from the IMU (`platform.rotation`) and dynamically offsetting `self.horizon` to keep the slice level with the horizon.

### C. Distance-Scaled Search Window
*   **The Issue:** A fixed pixel window size does not scale with distance. At $1.0\text{ m}$ distance, a $120\text{px}$ window represents about $0.26\text{ m}$ of physical width (ideal for Matty's $0.6\text{ m}$ footprint). However, if the leader pulls ahead to $2.5\text{ m}$, the leader's visual size shrinks, and a fixed $120\text{px}$ window would capture background elements (trees, walls), polluting the depth percentiles.
*   **Mitigation:** The search window width can be scaled dynamically relative to the measured distance to maintain a constant physical tracking width:
    $$\text{window\_width} = \max\left(40, \, \min\left(160, \, \text{round}\left(120 \cdot \frac{1.0}{\text{measured\_distance\_m}}\right)\right)\right)$$

### D. Direct Sun Glare on Active IR
*   **The Issue:** Direct high-altitude sunlight can overpower the OAK-D Pro's active IR dot projector, causing temporary depth map "blind spots" (returning a depth value of $0$).
*   **Mitigation:** While passive stereo matching still works on high-contrast physical features, our state machine implements **Lock Retention**. If depth values drop to $0$ momentarily, speed is immediately decelerated to $0\text{ m/s}$ (safe fallback), but the target lock memory is held for $10$ frames ($1.0\text{ s}$), allowing smooth tracking resumption once the glare passes.

---

## 6. Lessons Learned (June 8, 2026)

Following the first real-world field test of the `FollowRobot` module (`m05-matty-follow-robot-260608_183003.log`), we identified critical mathematical and physical relationships:

### A. Horizon Pitch Correction Sign Alignment
*   **The Bug:** The initial implementation used a positive correction sign (`+=`) for IMU pitch:
    $$\text{current\_horizon} = \text{horizon} + \text{pitch\_offset}$$
    While standing still (pitch $\approx +5.9^\circ$), this offset added $+53\text{px}$ to the horizon, setting it to $253\text{px}$, which successfully targeted the leader's rear body. However, as the robot accelerated and pitched **up** (nose-rise, pitch decreasing to $+3.3^\circ$), the correction shifted the slice **up** in the frame (smaller Y coordinate, e.g., $230\text{px}$), while the leader actually shifted **down** (higher Y coordinate, e.g., $270\text{px}$). This caused a complete target loss at $10.23\text{ s}$ as the sensor slice looked over the leader's head.
*   **The Fix:** Changed the pitch correction sign to subtraction (`-=`):
    $$\text{current\_horizon} = \text{horizon} - \text{pitch\_offset}$$
    This correctly couples pitch with image coordinates: when the nose rises (camera tilts up / pitch decreases), the target's pixel Y coordinate increases, and our search slice automatically descends to track it.

### B. Horizon Baseline Calibration
*   **Calibration Insight:** The default `horizon` config value (representing the slice coordinate when the camera is level at $0.0^\circ$ pitch) must be calibrated to **`300`**.
*   This baseline ensures that when the robot is standing still at its nominal mounting pitch ($+5.93^\circ$), the dynamic horizon evaluates to:
    $$\text{current\_horizon} = 300 - (5.93 \cdot 9.09) \approx 246\text{px}$$
    This perfectly aligns the horizontal slice with the center of the leading robot's chassis.

### C. Validation Summary
*   Replaying the test log using `osgar.replay -F --config followme/config/matty-follow-robot.json` confirmed that **the robot now achieves 100% stable tracking for the entire duration of the log file**, surviving sudden stops, dynamic pitching, and sharp steering maneuvers without a single track loss.

