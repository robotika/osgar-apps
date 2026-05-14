# Plan: Road Following with Depth-based Obstacle Avoidance

This plan outlines the steps to integrate OAK-D depth data into the `robotem-rovne` navigation system to avoid collisions while following the road.

## 1. Research and Data Analysis
- **Analyze Depth Data**: Verify the format and resolution of the `oak.depth` stream in `m04-matty-redroad-260501_105531.log`.
- **Coordinate Alignment**: Confirm the spatial alignment between `oak.color` (and thus `oak.nn_mask`) and `oak.depth`.
- **Obstacle Identification**: Identify the specific point in the log where the tree collision occurs and check how it appears in the depth data.

## 2. Tooling and Visualization
- **Enhanced Visualization**: Create or update a script (based on `view_mask.py`) to visualize:
    - The road segmentation mask.
    - The depth map (as a heatmap).
    - Obstacles detected within a certain safety distance.
- **Bird's Eye View (BEV)**: Project depth and road mask into a top-down view to better plan steering.

## 3. Algorithm Development (Depth-NN Fusion)
- **Depth-based Obstacle Mask**:
    - Filter depth data for points closer than `stop_dist` or `warning_dist`.
    - Handle "noise" in depth data (e.g., median filter or min-pooling).
- **Safe Road Extraction**:
    - Project the NN mask and Depth data into the same coordinate space.
    - `safe_mask = nn_mask & (depth > safety_threshold)`.
- **Steering Strategy**:
    - Use `safe_mask` for `mask_center` calculation.
    - If `safe_mask` is too small (no path), set speed to 0.

## 4. Implementation in `RobotemRovne` Node
- **Update `on_depth`**: Implement the `on_depth` handler in `robotem-rovne/main.py`.
- **State Management**: Track the latest depth and NN mask to compute navigation commands.
- **Config Update**: Update `matty-redroad.json` to link `oak.depth` to `app.depth`.

## 5. Verification and Testing
- **Reproduce Failure**: Run `osgar.replay` on the original log to confirm the baseline behavior (collision).
- **Verify Logic Change**: Run `osgar.replay` without `-F` and expect failure due to changed behavior.
- **Validate Solution**: Run `osgar.replay -F` and verify that the robot now avoids the tree while staying on the road.

## 6. Refinement
- Tune `stop_dist` and `turn_angle` based on replay results.
- Add logging/prints to debug the fusion of road and depth data.
