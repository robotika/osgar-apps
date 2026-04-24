# OAK-D Color and Depth Calibration & Alignment

This document tracks research and implementation efforts for validating and improving the alignment between "oak.color" and "oak.depth" data in the `rerun-route` project.

## Current State
*   **Intrinsics:** Fixed hardcoded values in `main.py` and `extract_route_images.py` (fx=1400, fy=1400, cx=960, cy=540).
*   **Alignment:** Simple linear scaling between color frame dimensions (1920x1080) and depth frame dimensions (usually 640x400 or similar).
*   **3D Projection:** Basic $Z = depth / 1000$, $X = (u - cx) \cdot Z / fx$, $Y = (v - cy) \cdot Z / fy$.
*   **Matching:** ORB descriptors matched via BFMatcher or FLANN.
*   **Pose Estimation:** `solvePnPRansac` used for initial alignment.

## Goals
1.  **Offline Validation:** Create tools to process existing logs and quantify alignment error (e.g., reprojection error).

## Research Topics

### 1. ORB + solvePnP Validation
*   **Consistency Check:** If we have multiple frames with overlapping views, do the 3D points from different frames agree on the same world coordinates?
*   **Reprojection Error:** How well do the 3D points project back onto the 2D image after PnP? High error indicates poor calibration.


## Visual Verification Ideas
*   **3D Feature Plot:** 3D scatter plot of matched features to see if they form a coherent structure.
*   **Re-projection Viz:** Draw circles for original 2D keypoints and crosses for projected 3D points.
  This visualization is a diagnostic tool used to "see" the calibration error directly on the image. It works by
  comparing where a feature actually is versus where the 3D math thinks it should be.

  Here is the step-by-step breakdown:

  1. The Components
   * The Circle (Observation): This is the raw 2D feature detected by ORB on the current color image. It represents the
     "ground truth" of what the camera sees at that exact pixel.
   * The Cross (Projection):
       1. We take a 2D feature from a previous frame (or a reference landmark).
       2. We use the depth data to turn it into a 3D point $(X, Y, Z)$.
       3. We use our current pose2d (and joint_angle) to calculate where the camera is now.
       4. We mathematically project that 3D point back onto the current 2D image plane using our intrinsics. This result
          is the Cross.

  2. How to Read the Results
  By looking at the distance and direction between the Circle and its corresponding Cross, you can diagnose specific
  problems:

   * Perfect Overlap: The calibration is perfect. The 3D model of the world matches the 2D image perfectly.
   * Systematic Shift (All crosses are 5px to the right): This indicates an Extrinsic/Alignment error. The depth sensor
     and color sensor have a slight horizontal offset that isn't accounted for in the math.
   * Radial Scaling (Crosses are "further out" from the center than circles): This indicates an Intrinsic error (wrong
     focal length $f_x, f_y$). The math thinks the lens is "wider" or "narrower" than it actually is.
   * Jitter/Scattering: If circles and crosses are randomly far apart, it suggests high noise in the depth data or that
     the feature is on a "depth edge" (where the depth value is unstable).
   * Articulated "Swing": If you turn the robot's joint and the crosses "swing" away from the circles, it confirms that
     the lever-arm distance from the joint to the camera is measured incorrectly.


## Articulated Kinematics & Front-Part Extrinsics

Articulated robots (like Matty/m03) have a center pivot joint. Since the OAK-D camera is mounted on the front section, its world pose depends on both the platform pose and the steering joint angle.

### 1. The Kinematic Challenge
*   **Variable Offset:** Even at a fixed $(x, y, \theta)$, changing the `platform.joint_angle` $(\phi)$ swings the camera in a physical arc.
*   **Transformation Chain:**
    $$T_{world \to camera} = T_{world \to joint} \times T_{joint \to front}(\phi) \times T_{front \to camera}$$


## Test Variants for Individual Traces

These variants are proposed for validating calibration on a single logfile (trace) containing `platform.pose2d`, `oak.color`, and `oak.depth`.

### Variant 1: Reprojection Error Consistency (Point-based)
Focuses on the mathematical stability of the 3D-to-2D projection.
*   **Method:** Extract ORB features from `color`, look up 3D coordinates in `depth`, and use `solvePnPRansac` between frames.
*   **Validation Metric:**
    1.  **Average Reprojection Error:** Target < 1.5-2.0 pixels.
    2.  **Inlier Ratio:** High rejection suggests depth-color misalignment.
*   **Target:** Detecting incorrect focal length or principal point.
