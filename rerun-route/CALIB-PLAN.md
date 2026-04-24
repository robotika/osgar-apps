# Calibration Improvement Plan (CALIB-PLAN.md)

This document outlines the strategy for improving camera-to-robot alignment and intrinsic calibration for the `rerun-route` project, as researched in `CALIB.md`.

## 1. Objectives
*   **Quantify Error:** Establish a baseline reprojection error using existing logs.
*   **Improve Alignment:** Reduce the systematic shift between OAK-D color and depth data.
*   **Refine Intrinsics:** Replace hardcoded "approximate" values with calibrated ones.
*   **Articulated Kinematics:** (If applicable) Account for the center pivot joint in camera pose calculations.

## 2. Acceptance Criteria (AC)
*   **AC1:** Reprojection error of ORB features matched between frames is $< 2.0$ pixels (average).
*   **AC2:** Visual verification (Circle/Cross plot) shows no systematic pixel shift across the image.
*   **AC3:** Inlier ratio for `solvePnPRansac` remains $> 60\%$ during typical driving.
*   **AC4:** A validation script exists that can process any `.log` file and output a calibration report.

## 3. Implementation Options

### Option A: Manual Refinement & Validation (Low Effort)
Focus on verifying and manually tuning the hardcoded parameters in `main.py`.
*   **Pros:** Quick to implement; requires no new complex algorithms.
*   **Cons:** Not scalable; might miss non-obvious alignment issues.
*   **Tasks:**
    1.  Create `validate_calibration.py` to calculate reprojection error on a log.
    2.  Manually adjust `fx, fy, cx, cy` to minimize error.
    3.  Add a fixed extrinsic offset $(dx, dy)$ between color and depth if a shift is detected.

### Option B: Automated Optimization (Medium Effort)
Use optimization (e.g., `scipy.optimize.minimize`) to find the best camera parameters.
*   **Pros:** More accurate; adapts to different OAK-D units.
*   **Cons:** Requires clean logs with good feature coverage.
*   **Tasks:**
    1.  Implement a cost function that takes `(fx, fy, cx, cy, extrinsics)` and returns total reprojection error.
    2.  Run optimization on selected reference logs.
    3.  Update `main.py` to optionally load these parameters from a JSON config.

### Option C: Kinematic-Aware Calibration (High Effort)
Integrate the robot's articulation (`joint_angle`) into the pose estimation.
*   **Pros:** Essential for articulated robots like Matty/m03 to maintain accuracy during turns.
*   **Cons:** Higher complexity; requires precise physical measurements of the robot.
*   **Tasks:**
    1.  Define the transformation chain: $T_{world \to camera} = T_{world \to joint} \times T_{joint \to front}(\phi) \times T_{front \to camera}$.
    2.  Measure/Calibrate the "lever arm" from joint to camera.
    3.  Implement kinematic-aware projection in the validation tool.

## 4. Proposed Workflow
1.  **Phase 1 (Diagnostic):** Build the `validate_calibration.py` tool. It should produce the Circle/Cross visualization mentioned in `CALIB.md`.
2.  **Phase 2 (Optimization):** Use Option B to find better constants.
3.  **Phase 3 (Integration):** Update `RerunRoute` in `main.py` to use the new parameters and optionally the kinematic model.

## 5. Tools to be Created/Modified
*   `rerun-route/validate_calibration.py`: New diagnostic tool.
*   `rerun-route/config/calibration.json`: New storage for optimized parameters.
*   `rerun-route/main.py`: Updated to use external calibration and handle joint angles.
