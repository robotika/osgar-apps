# Mosaic Validation Strategy & Ideas

This document outlines structural, geometric, and radiometric strategies to validate the accuracy, consistency, and visual quality of the stitched global BEV (Bird's Eye View) road mosaic images.

---

## 1. Geometric & Spatial Validation

To ensure the physical dimensions on the mosaic represent real-world coordinates accurately, we can apply several spatial verification methods.

### A. Road Width Consistency
- **Concept:** The road is calibrated to have a physical width ($W_{\text{road}}$) in meters (e.g., 2.0m). If the homography and affine transformations are correct, the road width in the stitched global mosaic should remain constant and measure exactly:
  $$P_{\text{width}} = \frac{W_{\text{road}}}{\text{resolution}} \quad (\text{pixels})$$
- **Validation Method:** 
  1. Slice the global mosaic perpendicular to the robot's trajectory at fixed distance steps (e.g., every 1.0m along the trajectory).
  2. Detect the left and right lane boundaries (using thresholding, edge detection, or peak intensity of lane marking filters).
  3. Measure the distance in pixels between the boundaries.
  4. **Metric:** Compute the Mean, Standard Deviation, and Root Mean Squared Error (RMSE) of the measured width against $P_{\text{width}}$. Any significant deviation or high variance indicates perspective or calibration drift.

### B. Trajectory Length Verification
- **Concept:** If the robot drove a straight or curved path of known length $L$ (calculated as the cumulative distance of the `Pose2D` stream), the corresponding distance on the mosaic must scale exactly.
- **Validation Method:**
  1. Extract the start point $(X_{\text{start}}, Y_{\text{start}})$ and end point $(X_{\text{end}}, Y_{\text{end}})$ of the trajectory in the global canvas pixel coordinates.
  2. Measure the physical path length directly along the drawn path on the image:
     $$\text{Length}_{\text{pixels}} = \sum_{i} \sqrt{(x_{i+1} - x_i)^2 + (y_{i+1} - y_i)^2}$$
     $$\text{Length}_{\text{meters}} = \text{Length}_{\text{pixels}} \times \text{resolution}$$
  3. **Metric:** Compare $\text{Length}_{\text{meters}}$ against the odometry-reported cumulative distance. High errors indicate slip, scaling issues, or incorrect coordinate transformations.

### C. Straightness of Known Straight Segments
- **Concept:** If the robot is known to have driven on a straight road segment, the lane lines in the mosaic should form perfect parallel straight lines.
- **Validation Method:**
  1. Run a Hough Transform (`cv2.HoughLines` or `cv2.HoughLinesP`) on a segment of the mosaic known to be straight.
  2. **Metric:** Verify that the detected lines are parallel (slopes match within a very tight tolerance) and have a variance in curvature close to zero.

---

## 2. Stitching & Alignment Quality (Overlaps)

Because the global canvas is constructed by sequentially overlaying images with heavy overlaps, mismatches in calibration or pose will manifest as "ghosting", double-vision, or seam lines.

### A. Feature-Based Continuity (Structural Similarity)
- **Concept:** If the perspective warping and poses are perfectly aligned, a distinct physical landmark on the road (such as a crack, stone, or line feature) should align exactly when overlapping frames are blended.
- **Validation Method:**
  1. For any two adjacent/overlapping frames, extract their shared overlapping region.
  2. Compute keypoints and descriptors (e.g., using SIFT, ORB, or AKAZE) on both raw frames.
  3. Warp the keypoint locations from both frames into the global mosaic coordinate space.
  4. Measure the distance (in pixels or meters) between matched keypoints.
  5. **Metric:** **Mean Match Displacement (MMD).** A mean displacement of < 2-3 pixels is excellent. Large displacements (e.g., > 10 pixels) highlight heading drift or homography inaccuracies.

### B. Overlap Structural Similarity (SSIM)
- **Concept:** Compare the local BEV images of consecutive frames in their overlapping area.
- **Validation Method:**
  1. Extract the overlapping polygon between Frame $N$ and Frame $M$.
  2. Project both onto a common local grid.
  3. **Metric:** Calculate the **Structural Similarity Index (SSIM)** and **Peak Signal-to-Noise Ratio (PSNR)** of the overlapping region. High similarity indicates excellent alignment.

---

## 3. Radiometric & Visual Quality

### A. Coverage Verification (No Gaps/Black Holes)
- **Concept:** The stitched trajectory should be a continuous ribbon. There should be no unrendered "black holes" (pixels with value 0,0,0) inside the path.
- **Validation Method:**
  1. Create a binary mask of the driven trajectory by dilating the robot coordinates by half the road width.
  2. Compute the intersection of this mask with the generated mosaic canvas.
  3. **Metric:** Count the number of unpainted (black) pixels within the trajectory mask. Ideally, coverage should be 100%.

### B. Seam Line Artifact Detection (Blending Smoothness)
- **Concept:** Hard edges at frame boundaries indicate sudden lighting shifts or exposure differences between frames.
- **Validation Method:**
  1. Apply a Sobel or Laplacian gradient filter to the output mosaic.
  2. Inspect the gradient magnitudes along the boundaries of individual stitched frames.
  3. **Metric:** High edge energy localized exactly at frame boundaries indicates poor blending or sudden exposure jumps. (Can be minimized in the future using multi-band blending or exposure compensation).

---

## 4. Dynamic Objects & Leading-Robot Occlusion

In convoy-style traces where the robot is actively following another vehicle, the leading platform will constantly occupy the center-forward portion of the camera's field of view, partially obscuring the road ahead. 

In a standard sequential overlay mosaic, this leads to **"ghost trails"**—where multiple stretched copies of the leading robot are continuously pasted onto the global canvas along the entire trajectory.

### A. The Challenge
- **Occlusion:** The leading robot blocks the view of actual road features (lines, lanes, surface changes) directly in front of the camera.
- **Stretching:** Because the leading robot is far away (near the horizon / top of the trapezoid), the perspective warp stretches its pixels heavily in the BEV image, turning a small vehicle into a long, distorted blob.

### B. Validation & Detection of Occlusion Artifacts
1. **Traveled Corridor Color Anomaly Detection:**
   - *Method:* Analyze the central column of the local BEV images (where the leading robot is expected to be). Compare the color histogram or variance of this region against known road surface colors (e.g., gray pavement or green grass).
   - *Metric:* Peak deviation score. Large spikes in color variation or non-road colors (e.g., chassis red, yellow, black) indicate occlusion.
2. **Keypoint Matching Rejection:**
   - *Method:* High-frequency, dynamic features on the moving leading robot should not be used as static ground keypoints for alignment.
   - *Metric:* Filter out keypoint matches that lie within the central occlusion zone. If matches within this zone show inconsistent vectors compared to outer (static road boundary) keypoints, they are flagged as dynamic occlusion.

### C. Mitigation Strategies to Document/Test
- **Near-Field Cropping (Horizon Masking):** 
  Since the leading robot is far ahead, it occupies the top region of the perspective trapezoid. Lowering the `Top Y` parameter or cropping out the top 20-30% of the BEV image can exclude the leading robot completely from the warped output, ensuring only the clean, un-obscured road closer to the robot is stitched.
- **Dynamic Masking (Segmentation):**
  Use a simple color-thresholding mask (or deep-learning models like YOLO) to segment the preceding robot, creating a binary exclusion mask. Set these pixels to transparent/black `(0, 0, 0)` so they do not get drawn onto the canvas.
- **Temporal Minimum/Median Blending:**
  Instead of simple overwriting (where the robot's pixel always wins), keep a history of overlapping frames. If the leading robot's distance changes or it weaves slightly, a temporal median or minimum filter across overlapping pixels can filter out the transient vehicle pixels, leaving the static road underneath.

---

## 5. Advanced Real-World Modeling: Varying Widths, Lateral Offsets, and 3D Obstacles

In unstructured outdoor environments, several physical assumptions made in the basic PoC begin to break down:
1. **The road width is not constant** (it widens, narrows, or merges).
2. **The robot is not centered** (lateral weaving and offset).
3. **The environment is 3D** (objects like trees, poles, and walls extend vertically and violate the flat-plane homography assumption).

Below are the mathematical challenges and concrete engineering strategies to validate and handle these real-world variations.

### A. Varying Road Widths
- **Challenge:** If calibration assumes a fixed road width to compute the global scaling, any widening of the road will be incorrectly warped as if it were still the default width, or will result in coordinate stretching.
- **Solution & Validation:**
  - **Intrinsic Ground Resolution (GSD):** Instead of using road width to compute `meters_per_pixel`, calculate the **Ground Sampling Distance (GSD)** based purely on physical camera parameters (sensor pitch, focal length, mounting height, and pitch angle). GSD is a physical invariant of the camera mount and does not change based on road width.
  - **Dynamic Boundary Extraction:** Let the lane lines in the mosaic widen and narrow naturally. In the validation phase, verify that the *local ground scale* (meters per pixel) remains perfectly constant, even when the distance between detected lane boundaries changes.

### B. Lateral Offsets & Weaving (Robot is not Centered)
- **Challenge:** The robot weaves, drives off-center, or turns, causing the road to shift sideways in the camera's view.
- **Solution & Validation:**
  - **Rigid Body Extrinsics (Camera-to-Robot Transform):** If the camera is mounted with a physical offset relative to the robot's center of rotation, apply a static extrinsic rigid transform matrix $T_{\text{camera}\to\text{robot}} = (x_{\text{offset}}, y_{\text{offset}}, \theta_{\text{offset}})$ to map the BEV coordinates into the robot center *before* applying the global `Pose2D` odometry transform.
  - **Closed-Loop Alignment Validation:** Since the 2D affine transform rotates and translates the entire BEV patch based on the robot's heading and position, the road's global coordinates should remain perfectly static on the canvas while the robot's trajectory line moves left or right relative to it. Verify this by ensuring the road edges form continuous global curves regardless of the robot's lateral maneuvers.

### C. 3D Non-Plane Objects (Trees, Walls, Posts)
- **Challenge (Radial Distortion/Smearing):** Homography operates on the strict assumption of a **flat ground plane (2D)**. Any 3D object extending vertically above the ground (e.g., trees, lampposts, walls) violates this. When warped, vertical 3D coordinates are projected onto the ground plane, causing them to stretch out infinitely as **radial streaks/smears** pointing away from the camera's optical center.
- **Solution & Validation:**
  1. **Near-Field Cropping:** 
     3D radial distortion increases exponentially with distance (towards the horizon). By aggressively cropping the top portion of the BEV frame (limiting the forward look-ahead distance to 3-5 meters), 3D smearing is minimized because only the flat ground immediately in front of the robot is processed.
  2. **Semantic Masking (Segmentation):**
     Use a lightweight segmentation model (like YOLOv8-seg or Fast-SCNN) to detect non-road classes (e.g., `tree`, `building`, `sky`, `trunk`, `pole`). Generate a binary exclusion mask for these classes and set their pixels to transparent `(0, 0, 0)` prior to perspective warping.
  3. **Depth-Based Plane Segmentation (OAK-D integration):**
     Since the OAK camera provides stereo depth/disparity, we can convert the pixels into a 3D point cloud:
     - Fit a dominant ground plane equation $Ax + By + Cz + D = 0$ using **RANSAC**.
     - Identify any 3D points where the height $|Ax + By + Cz + D| > h_{\text{threshold}}$ (e.g., more than 5cm above the ground).
     - Generate a binary mask for these high points (trees, obstacles) and exclude them from the BEV stitching process. This guarantees that only flat ground is drawn on the mosaic.

---

## 6. Proposed Validation Tooling

To put these ideas into practice, we propose creating an automated tool: `validate_mosaic.py`.

```bash
uv run python bevroad/validate_mosaic.py <mosaic-image-path> --csv <overview-csv-path> --resolution 0.02
```

### Planned Features:
1. **Interactive Landmark Measurer:** Allows the user to click two points on the mosaic, and displays the real-world distance in meters based on the pixel distance and canvas resolution.
2. **Automated Lane-Width Profile:** Plots a chart showing the width of the road (in meters) along the entire length of the trajectory.
3. **Alignment Drift Report:** Computes ORB matching between sequential frames in the mosaic and reports the average drift error in centimeters.
