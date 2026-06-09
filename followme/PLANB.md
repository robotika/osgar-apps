# Alternative Robot-Following Plan: 2D Spatial Depth Clustering (Plan B)

This document outlines an alternative, more robust depth-only robot-following approach (**Plan B**) designed to operate completely independently of the camera's IMU pitch feedback. 

---

## 1. Context & Motivation

In the initial implementation (Plan A, detailed in `followme/PLAN.md`), target tracking relied on a narrow horizontal slice of the depth image ($60\text{ px}$ height) centered on a dynamically calculated horizon line. 
While this approach is computationally efficient, real-world testing exposed a critical vulnerability:
*   **IMU Pitch Sensitivity:** Small physical changes in the robot's pitch (due to acceleration, deceleration, uneven terrain, or mounting vibrations) cause the target to quickly drift entirely out of the narrow $60\text{ px}$ window. 
*   **Calibration & Signal Dependency:** Relying on IMU pitch (`platform.rotation`) adds software complexity, requiring precise sign alignment, zero-offset calibration, and a highly responsive IMU stream. If the IMU is noisy or slightly uncalibrated, the tracking easily fails.

**Plan B** removes the IMU dependency entirely. Instead of searching a narrow 1D horizontal strip, we analyze a **wide 2D Region of Interest (ROI)** and segment the leading robot directly in 2D depth-space using **Connected-Component Clustering and Physical Dimension Filtering**.

---

## 2. Technical Approach: 2D Spatial Depth Clustering

By expanding our search space to a large vertical band, the leading robot remains fully within our tracking window regardless of sudden pitch variations. We isolate the robot by filtering pixels that match its 3D physical size and distance.

```
       Original Depth Frame (640x400)
┌──────────────────────────────────────────────┐
│                                              │
│    Wide ROI (120 to 360 px)                  │
│    ┌────────────────────────────────────┐    │
│    │  [Background - Far/Invalid Depth]  │    │
│    │                                    │    │
│    │         ┌───────────┐              │    │
│    │         │  Leader   │ (Contiguous  │    │
│    │         │  Cluster  │  Blob)       │    │
│    │         └───────────┘              │    │
│    │  [Ground - Filtered out]           │    │
│    └────────────────────────────────────┘    │
└──────────────────────────────────────────────┘
```

### Step 1: Define a Wide 2D Region of Interest (ROI)
Instead of a $60\text{ px}$ band, we look at a wide vertical range:
*   **Vertical Bounds ($Y$):** $y \in [120, 360]$ (excluding the top $120\text{ px}$ to ignore tree tops/ceilings and the bottom $40\text{ px}$ to ignore the immediate ground plane right in front of our bumper).
*   **Horizontal Bounds ($X$):** $x \in [80, 560]$ (excluding peripheral columns to ignore side obstacles/clutter).

This $480 \times 240\text{ px}$ search window is massive. A $\pm 10^\circ$ pitch variation shifts the target by only $\approx 90\text{ px}$, keeping it safely inside this window.

### Step 2: Distance Range Filtering
We create a binary mask of pixels that lie within the possible tracking distance of the leading robot:
$$\text{mask}[y, x] = \begin{cases} 
1 & \text{if } 500\text{ mm} \le \text{depth}[y, x] \le 3200\text{ mm} \\ 
0 & \text{otherwise} 
\end{cases}$$
This instantly filters out far-away background noise (trees, walls) and extremely close-range glare.

### Step 3: Connected-Component Labeling (Blob Detection)
Using OpenCV's `cv2.connectedComponentsWithStats` (or a high-performance numpy-based equivalent), we label contiguous clusters of pixels in the binary mask:
```python
num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_mask)
```
Each cluster $i$ (for $i \ge 1$) provides:
*   **2D Bounding Box:** Left ($x_{\min}$), Top ($y_{\min}$), Width ($w$), Height ($h$).
*   **Area (Pixels):** Total active pixels in the cluster.

### Step 4: 3D Physical Dimension Verification
To distinguish the leading robot from other objects (like a skinny pole, a wide flat wall, or a small patch of ground noise), we project the 2D bounding box of each cluster into **physical 3D dimensions** (width and height in meters) using camera intrinsics.

Given the cluster's **median depth** $D_{\text{median}}$ (meters) of all its non-zero pixels, and camera specifications (Horizontal FOV $\text{HFOV} = 69^\circ$, Vertical FOV $\text{VFOV} = 44^\circ$):
$$\text{phys\_w} = 2 \cdot D_{\text{median}} \cdot \tan\left(\frac{69^\circ}{2}\right) \cdot \frac{w}{640.0}$$
$$\text{phys\_h} = 2 \cdot D_{\text{median}} \cdot \tan\left(\frac{44^\circ}{2}\right) \cdot \frac{h}{400.0}$$

A candidate cluster is classified as the **leading robot** if it matches the physical constraints of Matty's chassis:
1.  **Physical Width:** $0.3\text{ m} \le \text{phys\_w} \le 0.8\text{ m}$
2.  **Physical Height:** $0.2\text{ m} \le \text{phys\_h} \le 0.7\text{ m}$
3.  **Aspect Ratio:** $0.5 \le \frac{\text{phys\_w}}{\text{phys\_h}} \le 2.0$ (filters out poles/lines).
4.  **Density/Area:** $\ge 200$ pixels (filters out sparse noise).

### Step 5: Target Continuity & Tracking Match
If multiple candidate clusters pass the physical filter:
*   **If tracking is active:** Choose the cluster whose 2D centroid $(x_{\text{centroid}}, y_{\text{centroid}})$ is closest to our last-known target position $(X_{\text{last}}, Y_{\text{last}})$.
*   **If looking for a new target:** Choose the cluster that is closest in distance ($D_{\text{median}}$) and closest to the frame's horizontal center ($320$).

---

## 3. Control Loop Integration

The chosen cluster's spatial parameters are fed directly into the P-controller:
*   **Measured Distance:** $D_{\text{measured}} = D_{\text{median}}$ (meters).
*   **Target X:** $X_{\text{target}} = \text{centroid\_x}$ (or the center of the bounding box $(x_{\min} + x_{\max}) / 2$).
*   **Steering Error:** $\theta_{\text{err}} = \frac{320 - X_{\text{target}}}{320} \cdot 34.5^\circ$
*   **Control Commands:**
    $$\text{speed} = \min(\text{max\_speed}, \, \max(0, \, K_{p\text{\_distance}} \cdot (D_{\text{measured}} - D_{\text{target}})))$$
    $$\text{steering} = K_{p\text{\_steering}} \cdot \theta_{\text{err}}$$

---

## 4. Why Plan B is Inherently Robust

1.  **Zero IMU Dependency:** Removes all requirements for IMU readings, mounting-angle pitch calibrations, or timing synchronizations.
2.  **Pitch-Immunity:** Because the search window is vertically wide ($240\text{ px}$), pitch variations merely move the target cluster up and down within the ROI. The cluster's physical size and 3D connectivity remain invariant.
3.  **Active Ground Plane Discrimination:**
    *   The immediate ground plane is cropped out of the ROI ($y > 360$).
    *   Farther ground patches are naturally filtered because they do not form compact, vertical 3D blobs (they have a continuous depth gradient and an extremely flat aspect ratio, failing the physical height and aspect ratio checks).
4.  **Automatic Scale Scaling:** Standard horizon-slice methods must manually scale the horizontal search window width relative to distance (equation in Plan A). In Plan B, **scale is handled automatically** because we segment the actual physical object bounds; the bounding box shrinks/grows organically with distance while keeping the exact target center.

---

## 5. Phased Implementation Roadmap & Analysis Automation

To facilitate offline analysis and fine-tuning, we integrate a dedicated **Diagnostic Image-Saving Trigger** specifically designed for use during log replays.

### A. Automated Replay Diagnostics (Option 1)
*   **Trigger Condition:** Diagnostic images are only saved when running in local replay mode (`osgar.replay`) with the `--verbose` flag active (i.e., `self.verbose` is `True`). This prevents the robot from writing files during real-world runs, avoiding storage or CPU bottlenecks.
*   **Output Directory:** Images are saved in a hardcoded local directory `debug_images/` (created automatically if missing).
*   **Alphabetical Timestamp Naming Convention:**
    To support perfect alphabetical sorting, retain chronological ordering, and cross-reference back to the source log file, we name files using the format:
    `debug_images/{short_log_name}_{centiseconds:06d}_{event_type}.jpg`
    *   `short_log_name`: The prefix/short name of the log run (e.g., `m05_183052`).
    *   `centiseconds`: Calculated as `round(self.time.total_seconds() * 100.0)`. Formatted as a zero-padded 6-digit integer (`{:06d}`), which covers up to `9999.99` seconds (over 166 minutes of continuous recording, far exceeding the 10-minute target) while maintaining absolute chronological and alphabetical alignment.
    *   `event_type`: A brief label indicating the tracking state (e.g., `tracked`, `lost`, `estop`).
    *   *Example filename:* `debug_images/m05_183052_012340_tracked.jpg` (representing a snapshot at exactly $123.40\text{ s}$ into the run).

### B. Overlay Visuals
Every saved diagnostic image will be annotated with:
1.  **ROI Boundaries:** A dotted bounding rectangle indicating the search ROI ($x \in [80, 560], y \in [120, 360]$).
2.  **Target Overlay:** A bright green box surrounding the matched cluster with overlay text showing physical dimensions (e.g., `Robot (W: 0.52m, H: 0.45m, D: 1.25m)`).
3.  **Rejected Overlays:** Soft red/yellow boxes around rejected clusters showing why they were discarded (e.g., `Rejected: W: 1.85m, H: 0.12m` - "Ground").

---

### Phase 1: Prototype Development
*   Implement `on_depth` in a new class/node (or as a configuration option in `follow_robot.py`).
*   Incorporate `cv2.connectedComponentsWithStats` for lightning-fast blob segmentation.
*   Implement 3D projection formulas to calculate physical width/height.
*   Implement the `debug_images` saving logic utilizing the padded centisecond naming format.

### Phase 2: Offline Log Verification (Replay)
*   Replay existing robot logs (e.g., `m05-matty-follow-robot-260608_173840.log`) with `osgar.replay --verbose` using the new algorithm.
*   Inspect the generated images in `debug_images/` to verify that the segmented target's physical size aligns perfectly with Matty's dimensions (~0.5m wide).
*   Verify that tracking remains 100% lock-on during periods of dynamic pitching.

### Phase 3: Unit Testing
*   Update `test_follow_robot.py` with mock depth maps containing 2D target shapes and verifying they are correctly clustered and identified as the target.

---

## 6. Coexistence Architecture with Plan A

To allow both algorithms to coexist seamlessly in the codebase and be selected via configuration, we separate **perception (target detection)** from **control (driving/steering)**. 

### A. Configuration-Driven Delegation
We introduce a new configuration parameter `algorithm` with two valid options:
*   `"slice"` (default - Plan A): Uses the horizon slice + IMU pitch adjustment.
*   `"clustering"` (Plan B): Uses 2D spatial connected component clustering.

### B. Code Structure (`follow_robot.py`)
Both algorithms share the same initialization, safety limits, LED states, and closed-loop control logic (`on_pose2d`). Inside `on_depth`, the node delegates depth-processing to the corresponding algorithm method:

```python
class FollowRobot(Node):
    def __init__(self, config, bus):
        # ... standard initialization ...
        self.algorithm = config.get('algorithm', 'slice')
        
        # Plan A specific configs
        self.horizon = config.get('horizon', 300)
        self.depth_height = config.get('depth_height', 60)
        
        # Plan B specific configs
        self.min_phys_width = config.get('min_phys_width', 0.3)
        self.max_phys_width = config.get('max_phys_width', 0.8)
        self.min_phys_height = config.get('min_phys_height', 0.2)
        self.max_phys_height = config.get('max_phys_height', 0.7)

    def on_depth(self, data):
        if self.algorithm == 'slice':
            target_x, distance = self.track_via_slice(data)
        elif self.algorithm == 'clustering':
            target_x, distance = self.track_via_clustering(data)
        else:
            raise ValueError(f"Unknown tracking algorithm: {self.algorithm}")

        if target_x is not None and distance is not None:
            self.last_target_x = target_x
            self.last_distance = distance
            self.last_target_time = self.time

        # ... common telemetry, LED updates, etc. ...
```

### C. Helper Method Responsibilities
Each tracking helper method is responsible for one specific task: taking the raw depth frame and returning the matched target's `(target_x, distance_meters)`.
*   `track_via_slice(self, data)`: Implements the current Plan A logic using IMU-adjusted horizontal band extraction and percentile estimation.
*   `track_via_clustering(self, data)`: Implements the Plan B logic using OpenCV `connectedComponentsWithStats`, physical size projections, and target proximity matching.

This clean separation ensures that switching between the two approaches is a matter of changing a single line in the JSON configuration file, without modifying any behavioral or safety-critical control logic.

