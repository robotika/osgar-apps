# OAK-D Color and Depth Calibration & Alignment

This document tracks research and implementation efforts for validating and improving the alignment between "oak.color" and "oak.depth" data in the `rerun-route` project.

## Current State
*   **Intrinsics:** Fixed hardcoded values in `main.py` and `extract_route_images.py` (fx=1400, fy=1400, cx=960, cy=540).
*   **Alignment:** Simple linear scaling between color frame dimensions (1920x1080) and depth frame dimensions (usually 640x400 or similar).
*   **3D Projection:** Basic $Z = depth / 1000$, $X = (u - cx) \cdot Z / fx$, $Y = (v - cy) \cdot Z / fy$.
*   **Matching:** ORB descriptors matched via BFMatcher or FLANN.
*   **Pose Estimation:** `solvePnPRansac` used for initial alignment.

## Goals
1.  **Retrieve Factory Calibration:** Extract and use the actual OAK-D calibration from the hardware or stored logs if available.
2.  **Proper Alignment:** Account for the physical offset between the RGB camera and the stereo pair (extrinsics).
3.  **Offline Validation:** Create tools to process existing logs and quantify alignment error (e.g., reprojection error).
4.  **Visual Verification:** Generate overlays (depth heatmaps on color, 3D point clouds) to manually inspect quality.

## Research Topics

### 1. OAK-D Intrinsics & Extrinsics
*   **Source:** DepthAI API can retrieve calibration data. Can we store this in the OSGAR log at the start?
*   **Resolution Dependency:** Intrinsics change with resolution and cropping. We need to ensure the parameters match the "oak.color" and "oak.depth" stream configurations.

### 2. Depth to Color Alignment
*   **Hardware Alignment:** OAK-D supports `setRegistrationConfig` or `setLeftRightCheck` to align depth to the RGB camera.
*   **Software Alignment:** If not aligned in hardware, we must use the extrinsics (rotation and translation between cameras) to project depth points into the RGB frame.
*   **FOV Differences:** RGB camera often has a different FOV than the stereo pair.

### 3. ORB + solvePnP Validation
*   **Consistency Check:** If we have multiple frames with overlapping views, do the 3D points from different frames agree on the same world coordinates?
*   **Reprojection Error:** How well do the 3D points project back onto the 2D image after PnP? High error indicates poor calibration.

### 4. Offline Analysis Workflow
*   **Log Batch Processing:** Script to iterate through a folder of logs, extract features, and report alignment statistics.
*   **Relative Consistency:** Since high-precision GPS or LIDAR is unavailable, "ground truth" must be established through relative metrics. This involves checking visual consistency (loop closure, feature persistence) against "relatively good" odometry.
*   **Area-Based Consistency & Cross-Log Validation:** Many recordings are likely from the same physical area (e.g., "redroad"). 
    *   **Systematic Bias:** By analyzing multiple logs from the same area, we can identify if calibration errors are consistent across runs, helping distinguish between sensor noise and systematic misalignment.
    *   **Cross-Log Matching:** Attempt to match ORB features and 3D points between different logs of the same area to verify that our projections remain stable across different time-of-day or lighting conditions.

## Implementation Plan

### Phase 1: Diagnostic Tool
*   [ ] Create `verify_alignment.py` that:
    *   Reads a log.
    *   Extracts color and depth frames.
    *   Generates a depth-overlay video/image series.
    *   Calculates reprojection error for a set of ORB matches.

### Phase 2: Calibration Source & Strategy
*   **Calibration via Configuration:** Store resolution-specific and robot-specific intrinsics in the application configuration. Follow the pattern used in the DTC project where parameters are prefixed by robot names (e.g., `"m03-intrinsics"`). This allows for easy overrides when factory data is unavailable in the log.
*   [ ] Update `rerun-route` to search for these robot-specific calibration keys.
*   [ ] Update OAK-D driver (in OSGAR) to log calibration data for future recordings.

### Phase 3: Advanced Alignment
*   [ ] Implement 3D-to-2D projection using full camera models (including distortion).
*   [ ] Support hardware-aligned depth if present in the log.

## Visual Verification Ideas
*   **Depth-on-Color Overlay:** Semi-transparent colorization of the RGB image based on depth values.
*   **3D Feature Plot:** 3D scatter plot of matched features to see if they form a coherent structure.
*   **Re-projection Viz:** Draw circles for original 2D keypoints and crosses for projected 3D points.

## Alternative Approaches

### 1. Stereo-First Approach (Bypassing RGB-Depth Alignment)
Instead of matching ORB features on the RGB image and then looking up depth, perform feature detection and matching directly on the synchronized Left and Right mono streams. 
*   **Goal:** Calculate 3D positions from stereo disparity of features.
*   **Advantage:** Bypasses RGB-to-Depth alignment and FOV matching issues.
*   **Caveat:** Current recordings do not include separate Left and Right mono streams. This approach is reserved for future data collections where these streams are enabled.

### 2. Edge-Based Alignment (Geometric Consistency)
Detect edges in the Color image (Canny/Sobel) and compare them with "depth edges" (where depth jumps occur).
*   **Goal:** Use correlation between edge maps to calculate a dynamic translation or scaling factor to "snap" the depth map to the color frame.
*   **Advantage:** Corrects based on actual data rather than relying solely on static factory intrinsics.

### 3. Structure from Motion (SfM) Refinement
Use a sequence of Color frames to build a sparse 3D point cloud using SfM (color only).
*   **Goal:** Compare the scale and structure of this cloud with the OAK-D Depth data to identify systematic warping or scale issues.

### 4. Mutual Information Alignment
Treat color and depth images as two different modalities and maximize the "Statistical Mutual Information" between them by adjusting camera parameters.
*   **Advantage:** Robust to lighting changes and noisy depth data.

### 5. Multi-Frame Depth Averaging (Temporal Filter)
Aggregate depth data over several frames (especially when moving slowly) to fill "holes" and reduce noise in the 3D points used for `solvePnP`.
*   **Advantage:** Improves reliability of the 3D matching component without needing better single-frame calibration.

## OAK-D API for Calibration Retrieval

To retrieve factory calibration data from an OAK-D Pro device, use the `depthai` Python API. The key class is `CalibrationHandler`, which can be accessed from a connected `Device`.

### 1. Basic Retrieval Example
```python
import depthai as dai
import numpy as np

with dai.Device() as device:
    calibData = device.readCalibration()
    
    # Cameras: CAM_A (RGB), CAM_B (Left), CAM_C (Right)
    # Note: Older API used CameraBoardSocket.RGB, LEFT, RIGHT
    rgb_socket = dai.CameraBoardSocket.CAM_A
    left_socket = dai.CameraBoardSocket.CAM_B
    
    # Intrinsics (3x3 Matrix)
    # Providing width/height scales the matrix to that resolution
    intrinsics = calibData.getCameraIntrinsics(rgb_socket, 1920, 1080)
    
    # Extrinsics (4x4 Transformation Matrix)
    # From RGB to Left camera
    extrinsics = calibData.getCameraExtrinsics(rgb_socket, left_socket)
    
    # Distortion (typically first 5 values for OpenCV)
    distortion = calibData.getDistortionCoefficients(rgb_socket)
```

### 2. Key Methods & Parameters
*   **`getCameraIntrinsics(socket, width, height)`**: Returns the 3x3 camera matrix. Scaling is critical because OSGAR often records at different resolutions (e.g., THE_1080_P vs THE_400_P) than the sensor's native resolution.
*   **`getCameraExtrinsics(src, dst)`**: Returns a 4x4 matrix representing the rotation and translation between two sensors. This is essential for projecting Depth (from stereo cameras) into the RGB coordinate system if they are not aligned in hardware.
*   **`getDistortionCoefficients(socket)`**: Returns distortion coefficients. For standard `cv2.undistort`, use the first 5 values `[k1, k2, p1, p2, k3]`.
*   **`getFov(socket)`**: Returns Horizontal FOV, useful for quick FOV matching checks.

### 3. Hardware Alignment (On-Device)
To simplify software alignment, OAK-D can perform RGB-Depth alignment on-the-fly:
```python
# During pipeline configuration:
stereo = pipeline.create(dai.node.StereoDepth)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A) # Align depth to RGB
```
*   **Verification:** If `setDepthAlign` was used during recording, the depth frame will already be in the RGB coordinate system, and `extrinsics` should not be applied manually.

## Articulated Kinematics & Front-Part Extrinsics

Articulated robots (like Matty/m03) have a center pivot joint. Since the OAK-D camera is mounted on the front section, its world pose depends on both the platform pose and the steering joint angle.

### 1. The Kinematic Challenge
*   **Variable Offset:** Even at a fixed $(x, y, \theta)$, changing the `platform.joint_angle` $(\phi)$ swings the camera in a physical arc.
*   **Transformation Chain:**
    $$T_{world \to camera} = T_{world \to joint} \times T_{joint \to front}(\phi) \times T_{front \to camera}$$
*   **Calibration Requirement:** We must precisely measure the fixed transform from the joint pivot to the camera optical center ($T_{front \to camera}$).

### 2. Proposed Extrinsic Calibration Methods

#### Method A: Kinematic Circle Fitting (Motion-based)
*   **Concept:** Drive the robot in a circle at a constant joint angle.
*   **Execution:** Fit the visual motion of features to the expected circular path. The deviation allows us to solve for the camera's radial and tangential offset from the joint.
*   **Benefit:** Accounts for actual mechanical play and "as-built" mounting.

#### Method B: Stationary Joint-Sweep (Geometric)
*   **Concept:** Observe a static ground target while sweeping the joint angle from max-left to max-right.
*   **Execution:** The target's path in the 3D depth space forms an arc. The radius of this arc is the horizontal distance from the joint to the target. By intersecting multiple arcs, we locate the joint center relative to the camera.

#### Method C: Re-localization Nulling (Global)
*   **Concept:** Move the joint, then manually reposition the robot base to restore the original camera view.
*   **Execution:** Use the difference in $(x, y, \theta)$ between the two visually identical states to solve for the $L_{joint \to cam}$ lever arm.

## Test Variants for Individual Traces

These variants are proposed for validating calibration on a single logfile (trace) containing `platform.pose2d`, `oak.color`, and `oak.depth`.

### Variant 1: Reprojection Error Consistency (Point-based)
Focuses on the mathematical stability of the 3D-to-2D projection.
*   **Method:** Extract ORB features from `color`, look up 3D coordinates in `depth`, and use `solvePnPRansac` between frames.
*   **Validation Metric:**
    1.  **Average Reprojection Error:** Target < 1.5-2.0 pixels.
    2.  **Inlier Ratio:** High rejection suggests depth-color misalignment.
*   **Target:** Detecting incorrect focal length or principal point.

### Variant 2: Depth-to-Color Edge Displacement (Edge-based)
Tests the spatial "sync" between the RGB sensor and the Depth sensor.
*   **Method:** Cross-correlation between `color` edges (Canny) and `depth` gradients.
*   **Validation Metric:**
    1.  **Alignment Offset (dx, dy):** Pixel shift required to maximize correlation.
    2.  **Correlation Strength:** Low strength indicates FOV mismatch or warping.
*   **Target:** Identifying physical extrinsics and FOV mismatches.

### Variant 3: 3D Plane Stability (Surface-based)
Uses robot movement to check if the 3D world stays "solid."
*   **Method:** Project dominant planes (ground/walls) into a global 3D system using `pose2d`.
*   **Validation Metric:**
    1.  **Planarity Error:** Flat surfaces should not appear curved or tilted as the robot moves.
    2.  **Texture Bleeding:** Texture projected onto 3D planes should not "slide" when viewing angles change.
*   **Target:** Validating depth scaling and odometry consistency.
