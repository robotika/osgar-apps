# Plan: Road Following with Depth-based Obstacle Avoidance

This plan outlines the steps to integrate OAK-D depth data into the `robotem-rovne` navigation system to avoid collisions while following the road.

## Status: Phase 1 (Reactive) Completed
- [x] Analyze Depth Data resolution and format.
- [x] Align Depth and NN Mask coordinate spaces.
- [x] Implement Tiered Avoidance: Slow down at 2.0m, Stop at 1.2m (after 5 frames).
- [x] Refine ROI (40-70% height, 40-60% width) to avoid ground/side triggers.
- [x] Validate via `osgar.replay`: Tree collision at 29.8s is now avoided (robot stops at 27.8s).

## 1. Research and Data Analysis
- **Analyze Depth Data**: Verified depth resolution (400x640) and NN mask resolution (112x112).
- **Coordinate Alignment**: Used `cv2.resize` with `INTER_NEAREST` to map depth to mask space.
- **Obstacle Identification**: Confirmed the tree appears at ~0.3m depth before collision.

## 2. Tooling and Visualization
- **Enhanced Visualization**: `visualize_collision.py` created to overlay NN mask and depth heatmap.
- **Analysis Script**: `analyze_data.py` created to inspect depth profiles in horizontal bands.
- **Bird's Eye View (BEV)**:
    - *Phase 1 (Reactive)*: COMPLETED. Direct projection to 1D steering and 0D speed control.
    - *Phase 2 (Mapping)*: PLANNED. Needed if obstacles fall into blind spots while turning.

## 3. Algorithm Development (Depth-NN Fusion)
- **Depth-based Obstacle Mask**: COMPLETED. Filters pixels closer than 2.0m from the road mask.
- **Tiered Speed Control**: COMPLETED.
    - `blocked_count > 0`: Speed = `max_speed / 2`.
    - `blocked_count >= 5`: Speed = 0.
- **Steering Strategy**: COMPLETED. Steering now uses the "depth-cleaned" road mask.

## 4. Implementation in `RobotemRovne` Node
- **Update `on_depth`**: COMPLETED. Caches latest depth frame.
- **Update `on_nn_mask`**: COMPLETED. Performs fusion and tiered detection.
- **Config Update**: COMPLETED. `matty-redroad.json` updated with `danger_dist` and `depth` links.

## 5. Verification and Testing (Standard Development Procedure)

### Step 5a: Local Replay Verification
```bash
uv run python -m osgar.replay robotem-rovne/data/m04-matty-redroad-260501_105531.log --module app --config robotem-rovne/config/matty-redroad.json
```

### Step 5b: Verification of Change (Expect Failure)
Confirmed: Replay fails/diverges due to early speed reduction and steering changes.

### Step 5c: Force Replay for Validation
```bash
uv run python -m osgar.replay robotem-rovne/data/m04-matty-redroad-260501_105531.log --module app --config robotem-rovne/config/matty-redroad.json -F
```
Verified: Robot stops 2 seconds before the original impact.

## 6. Refinement
- [ ] Test with "Successful" logs to ensure no regressions (false stops).
- [ ] Evaluate if Phase 2 (BEV Mapping) is required for sharp turns.
