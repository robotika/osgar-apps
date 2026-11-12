# Calibration Assessment Plan (CALIB-PLAN.md)

This document outlines the strategy for assessing the current state of camera-to-robot alignment and intrinsic calibration for the `rerun-route` project.

## 1. Primary Objective: "Know Where We Are"
Establish a baseline for calibration quality using existing hardcoded values and identify the nature of any alignment errors (systematic shift, scaling, or noise). This includes identifying the impact (or lack thereof) of the robot's articulation.

## 2. Acceptance Criteria (AC)
*   **AC1:** A diagnostic tool `validate_calibration.py` exists that can process an OSGAR `.log` file.
*   **AC2:** The tool reads `oak.color`, `oak.depth`, `platform.pose2d`, and `platform.joint_angle` (if available).
*   **AC3:** The tool calculates and prints the **Average Reprojection Error** (in pixels) for matched ORB features.
*   **AC4:** The tool generates a **Visual Verification Plot** (Circle/Cross) for at least 5 representative frames in the log.

## 3. Implementation Steps

### Step 1: Diagnostic Tool Development
Create `rerun-route/validate_calibration.py`. This tool will:
1.  Read `oak.color`, `oak.depth`, and `platform.pose2d`.
2.  Read `platform.joint_angle` for data completeness, but treat as `0.0` in projection math for Version 1 (assuming rigid body).
3.  Extract ORB features and match them between consecutive frames.
3.  Use the current hardcoded intrinsics ($f_x, f_y, c_x, c_y$) and depth data to project 3D points from Frame N to Frame N+1.
4.  Calculate the distance (error) between the projected points and the actual detected features in Frame N+1.

### Step 2: Baseline Assessment
Run the tool against a set of representative logs and document:
1.  Mean reprojection error.
2.  Visual observations: Do crosses consistently land to the left/right of circles? Do they scale incorrectly towards the edges?

## 4. Deliverables
*   `rerun-route/validate_calibration.py`: The assessment script.
*   A brief summary of the findings (Baseline Error & Error Type).
