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
