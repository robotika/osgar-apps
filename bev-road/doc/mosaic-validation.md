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

## 4. Proposed Validation Tooling

To put these ideas into practice, we propose creating an automated tool: `validate_mosaic.py`.

```bash
uv run python bev-road/validate_mosaic.py <mosaic-image-path> --csv <overview-csv-path> --resolution 0.02
```

### Planned Features:
1. **Interactive Landmark Measurer:** Allows the user to click two points on the mosaic, and displays the real-world distance in meters based on the pixel distance and canvas resolution.
2. **Automated Lane-Width Profile:** Plots a chart showing the width of the road (in meters) along the entire length of the trajectory.
3. **Alignment Drift Report:** Computes ORB matching between sequential frames in the mosaic and reports the average drift error in centimeters.
