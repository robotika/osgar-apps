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
- **Bird's Eye View (BEV)**: (Optional but recommended) Project depth and road mask into a top-down view to better plan steering.

## 3. Algorithm Development
- **Depth-based Obstacle Mask**: Generate an obstacle mask from the depth data by filtering for points closer than a threshold (e.g., `stop_dist`).
- **Safe Road Extraction**: Combine the `nn_mask` (road) with the depth-based obstacle mask.
    - `safe_road = road_mask AND (NOT obstacle_mask)`
- **Dynamic Steering**:
    - Calculate the desired direction based on the `safe_road` mask.
    - Implement an emergency stop or aggressive turn if an obstacle is detected directly in the path, even if the NN thinks it's a road.

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
