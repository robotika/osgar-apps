#!/usr/bin/env python3
"""
BEV Road Mapping - Consecutive Frame Pair Stitching & Alignment Validator.
Loads two sequential images, warps them to BEV space, and calculates
the geometric alignment displacement in meters.
"""

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

from bevroad.utils import global_to_local, parse_csv, transform_point
from bevroad.mosaic import load_calibration_data, calculate_spatial_parameters, get_bev_corners


def evaluate_pair(
    csv_path,
    index,
    config=None,
    road_width=2.0,
    lane_width_fraction=0.5,
    near=1.0,
    resolution=0.02,
    tolerance=15.0,
    no_vis=False,
    save_debug=False,
    verbose=True,
):
    """
    Evaluates the alignment quality of consecutive frames at `index` and `index + 1`.
    Prints detailed spatial and overlap analysis reports, including predictions of N+2.
    """

    def print(*args, **kwargs):
        if verbose:
            import builtins

            builtins.print(*args, **kwargs)

    csv_path = Path(csv_path)
    imdir = csv_path.parent

    # Parse records
    records = parse_csv(csv_path)
    if not records:
        print('Error: No valid records found in overview CSV.')
        sys.exit(1)

    assert index >= 0 and index + 2 < len(records), (
        f'Index {index} is out of bounds for {len(records)} records (0 to {len(records) - 3}).'
    )

    rec1 = records[index]
    rec2 = records[index + 1]

    # Paths to image files
    img1_path = imdir / rec1['filename']
    img2_path = imdir / rec2['filename']

    print(f'Analyzing consecutive pair: index {index} and {index + 1}')
    print(
        f'  Frame 1: {rec1["filename"]} at pose ({rec1["x"]:.3f}, {rec1["y"]:.3f}, heading={rec1["heading"]:.4f} rad)'
    )
    print(
        f'  Frame 2: {rec2["filename"]} at pose ({rec2["x"]:.3f}, {rec2["y"]:.3f}, heading={rec2["heading"]:.4f} rad)'
    )

    # Read images
    img1 = cv2.imread(str(img1_path))
    img2 = cv2.imread(str(img2_path))

    if img1 is None or img2 is None:
        print(f'Error: Failed to read image files: {img1_path} or {img2_path}')
        sys.exit(1)

    calib, config_path = load_calibration_data(imdir, config, verbose=verbose)

    M = np.array(calib['matrix_M'], dtype=np.float32)
    bev_w = calib['bev_width']
    bev_h = calib['bev_height']

    # Calculate local spatial parameters using helper
    m_per_pixel, bev_w_m, bev_h_m, d_near, d_far = calculate_spatial_parameters(
        road_width, bev_w, lane_width_fraction, bev_h, near
    )

    # 1. Warp both input images to local BEV space
    bev1 = cv2.warpPerspective(img1, M, (bev_w, bev_h))
    bev2 = cv2.warpPerspective(img2, M, (bev_w, bev_h))

    # Feature 1: Spatial & Size Analysis
    res = resolution
    patch_w_mosaic_px = bev_w_m / res
    patch_h_mosaic_px = bev_h_m / res
    patch_area_mosaic_px = patch_w_mosaic_px * patch_h_mosaic_px

    print('\nSpatial & Size Analysis:')
    print('-----------------------------------------')
    print(f'  BEV Local Resolution    : {m_per_pixel:.4f} m/pixel')
    print(f'  Mosaic Resolution       : {res:.4f} m/pixel')
    print(f'  BEV Patch Physical Size : {bev_w_m:.2f}m x {bev_h_m:.2f}m ({bev_w_m * bev_h_m:.2f} m²)')
    print(
        f'  BEV Patch in Mosaic     : {patch_w_mosaic_px:.1f} x {patch_h_mosaic_px:.1f} pixels'
        f' ({patch_area_mosaic_px:.1f} pixels)'
    )
    print('-----------------------------------------')

    # Feature 2: Percentage of Overlap of the Two Images
    cols, rows = np.meshgrid(np.arange(bev_w), np.arange(bev_h))
    xl1_all = d_near + (bev_h - 1 - rows) * m_per_pixel
    yl1_all = (bev_w_m / 2.0) - cols * m_per_pixel

    c1 = math.cos(rec1['heading'])
    s1 = math.sin(rec1['heading'])
    gx_all = rec1['x'] + xl1_all * c1 - yl1_all * s1
    gy_all = rec1['y'] + xl1_all * s1 + yl1_all * c1

    dx_all = gx_all - rec2['x']
    dy_all = gy_all - rec2['y']
    c2 = math.cos(rec2['heading'])
    s2 = math.sin(rec2['heading'])
    xl2_all = dx_all * c2 + dy_all * s2
    yl2_all = -dx_all * s2 + dy_all * c2

    row2_all = (bev_h - 1) - (xl2_all - d_near) / m_per_pixel
    col2_all = (bev_w_m / 2.0 - yl2_all) / m_per_pixel

    r2_all_int = np.round(row2_all).astype(int)
    c2_all_int = np.round(col2_all).astype(int)

    in_bounds2 = (r2_all_int >= 0) & (r2_all_int < bev_h) & (c2_all_int >= 0) & (c2_all_int < bev_w)

    overlap_pixels_raw = np.count_nonzero(in_bounds2)
    overlap_pct_raw = 100.0 * overlap_pixels_raw / (bev_w * bev_h)

    print('\nOverlap Analysis:')
    print('-----------------------------------------')
    print(f'  Raw Overlap Area        : {overlap_pixels_raw} pixels ({overlap_pixels_raw * (m_per_pixel**2):.4f} m²)')
    print(f'  Raw Overlap Percentage  : {overlap_pct_raw:.2f}% of full BEV image')
    print('-----------------------------------------')

    # Feature 3: Frame N+2 (Real Pose)
    rec3 = records[index + 2]
    pose3_x = rec3['x']
    pose3_y = rec3['y']
    pose3_heading = rec3['heading']

    print('\nFrame N+2 (Real Pose):')
    print('-----------------------------------------')
    print(f'  Actual N+2 pose         : ({pose3_x:.3f}, {pose3_y:.3f}, heading={pose3_heading:.4f} rad)')
    print('-----------------------------------------')

    # Check N+2 coverage
    dx_all_3 = gx_all - pose3_x
    dy_all_3 = gy_all - pose3_y
    c3 = math.cos(pose3_heading)
    s3 = math.sin(pose3_heading)
    xl3_all = dx_all_3 * c3 + dy_all_3 * s3
    yl3_all = -dx_all_3 * s3 + dy_all_3 * c3

    is_in_n2 = (
        (xl3_all >= d_near)
        & (xl3_all <= d_near + bev_h * m_per_pixel)
        & (yl3_all >= -bev_w_m / 2.0)
        & (yl3_all <= bev_w_m / 2.0)
    )

    remaining_overlap_mask = in_bounds2 & (~is_in_n2)
    remaining_overlap_pixels = np.count_nonzero(remaining_overlap_mask)
    remaining_overlap_area_sq_m = remaining_overlap_pixels * (m_per_pixel**2)

    # Pixel intensity comparison
    gray1 = cv2.cvtColor(bev1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(bev2, cv2.COLOR_BGR2GRAY)

    r1_indices, c1_indices = np.where(remaining_overlap_mask)
    if len(r1_indices) > 0:
        r2_indices = r2_all_int[r1_indices, c1_indices]
        c2_indices = c2_all_int[r1_indices, c1_indices]

        vals1 = gray1[r1_indices, c1_indices].astype(float)
        vals2 = gray2[r2_indices, c2_indices].astype(float)

        pixel_errors = np.abs(vals1 - vals2)
        pixel_mae = np.mean(pixel_errors)
        pixel_rmse = np.sqrt(np.mean(pixel_errors**2))

        # Tolerance check
        within_tol = pixel_errors <= tolerance
        pixels_within_tol = np.count_nonzero(within_tol)
        pct_within_tol = 100.0 * pixels_within_tol / len(r1_indices)
    else:
        pixel_mae = 0.0
        pixel_rmse = 0.0
        pixels_within_tol = 0
        pct_within_tol = 0.0

    # Remaining Overlap Analysis Report
    print('\nRemaining Overlap Analysis (After N+2 Overwrite):')
    print('-----------------------------------------')
    print(f'  Remaining Overlap Area  : {remaining_overlap_pixels} pixels ({remaining_overlap_area_sq_m:.4f} m²)')
    print(f'  Pixel Grayscale MAE     : {pixel_mae:.2f}')
    print(f'  Pixel Grayscale RMSE    : {pixel_rmse:.2f}')
    print(
        f'  Pixels Within Tolerance : {pixels_within_tol} / {len(r1_indices)}'
        f' ({pct_within_tol:.2f}%) [tolerance = {tolerance}]'
    )
    print('-----------------------------------------')

    # 5. Visualization
    bev_h_vis, bev_w_vis = bev1.shape[:2]

    # Create copies for drawing transparent colored overlays for tolerance visualization
    vis_bev1 = bev1.copy()
    vis_bev2 = bev2.copy()

    if len(r1_indices) > 0:
        # Create overlays
        green_overlay = np.array([0, 255, 0], dtype=np.uint8)
        red_overlay = np.array([0, 0, 255], dtype=np.uint8)

        # Indices within / outside tolerance in BEV1
        r1_win, c1_win = r1_indices[within_tol], c1_indices[within_tol]
        r1_out, c1_out = r1_indices[~within_tol], c1_indices[~within_tol]

        # Alpha blend on vis_bev1 (green for match, red for mismatch)
        alpha = 0.5
        vis_bev1[r1_win, c1_win] = (vis_bev1[r1_win, c1_win] * (1 - alpha) + green_overlay * alpha).astype(np.uint8)
        vis_bev1[r1_out, c1_out] = (vis_bev1[r1_out, c1_out] * (1 - alpha) + red_overlay * alpha).astype(np.uint8)

        # Indices within / outside tolerance in BEV2
        r2_win, c2_win = r2_indices[within_tol], c2_indices[within_tol]
        r2_out, c2_out = r2_indices[~within_tol], c2_indices[~within_tol]

        # Alpha blend on vis_bev2
        vis_bev2[r2_win, c2_win] = (vis_bev2[r2_win, c2_win] * (1 - alpha) + green_overlay * alpha).astype(np.uint8)
        vis_bev2[r2_out, c2_out] = (vis_bev2[r2_out, c2_out] * (1 - alpha) + red_overlay * alpha).astype(np.uint8)

    # Side by side concatenation
    vis_img = np.zeros((bev_h_vis, bev_w_vis * 2, 3), dtype=np.uint8)
    vis_img[:, :bev_w_vis] = vis_bev1
    vis_img[:, bev_w_vis:] = vis_bev2

    # Project N+2 corners back to BEV1/BEV2 pixels for drawing
    local_corners_3 = get_bev_corners(d_near, d_far, bev_w_m)
    global_corners_3 = []
    for lx, ly in local_corners_3:
        gx, gy = transform_point(lx, ly, pose3_x, pose3_y, pose3_heading)
        global_corners_3.append((gx, gy))

    pts_bev1 = []
    for gx, gy in global_corners_3:
        xl1_c, yl1_c = global_to_local(gx, gy, rec1['x'], rec1['y'], rec1['heading'])
        row1_c = (bev_h - 1) - (xl1_c - d_near) / m_per_pixel
        col1_c = (bev_w_m / 2.0 - yl1_c) / m_per_pixel
        pts_bev1.append([int(col1_c), int(row1_c)])

    pts_bev2 = []
    for gx, gy in global_corners_3:
        xl2_c, yl2_c = global_to_local(gx, gy, rec2['x'], rec2['y'], rec2['heading'])
        row2_c = (bev_h - 1) - (xl2_c - d_near) / m_per_pixel
        col2_c = (bev_w_m / 2.0 - yl2_c) / m_per_pixel
        pts_bev2.append([int(col2_c), int(row2_c)])

    pts_bev1_arr = np.array(pts_bev1, dtype=np.int32).reshape((-1, 1, 2))
    pts_bev2_arr = np.array(pts_bev2, dtype=np.int32).reshape((-1, 1, 2))

    pts_bev2_vis = pts_bev2_arr.copy()
    pts_bev2_vis[:, :, 0] += bev_w_vis

    # Draw actual N+2 boundary as a cyan line
    cv2.polylines(vis_img, [pts_bev1_arr], isClosed=True, color=(255, 255, 0), thickness=2)
    cv2.polylines(vis_img, [pts_bev2_vis], isClosed=True, color=(255, 255, 0), thickness=2)

    # Overlay Text stats
    cv2.putText(
        vis_img,
        f'Frame {index} BEV | Cyan: Actual N+2',
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        vis_img,
        f'Frame {index + 1} BEV | Green(<=tol), Red(>tol)',
        (bev_w_vis + 10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        vis_img,
        f'Raw Overlap: {overlap_pct_raw:.1f}% | RMSE: {pixel_rmse:.1f} |'
        f' In-Tol: {pct_within_tol:.1f}% (tol={tolerance})',
        (10, bev_h_vis - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )

    # Save to disk if requested
    if save_debug:
        debug_out_path = imdir / f'alignment_check_{index}_{index + 1}.png'
        cv2.imwrite(str(debug_out_path), vis_img)
        print(f'Saved match visualization to: {debug_out_path}')

    # Display window
    if not no_vis:
        win_name = 'BEV Alignment Validator (Consecutive Frame Pair)'
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.imshow(win_name, vis_img)
        print('Press any key in the GUI window to exit...')
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    metrics = {
        'overlap_pct_raw': overlap_pct_raw,
        'remaining_overlap_pixels': remaining_overlap_pixels,
        'pixel_mae': pixel_mae,
        'pixel_rmse': pixel_rmse,
        'pct_within_tol': pct_within_tol,
    }

    return vis_img, metrics


def main():
    from bevroad.mosaic import add_common_arguments

    parser = argparse.ArgumentParser(description='Verify stitching & alignment quality of two consecutive frames')
    parser.add_argument('csv_path', help='Path to overview.csv metadata file')
    parser.add_argument('index', type=int, help='Index of the first image (0-based)')

    add_common_arguments(parser, include_tolerance=True)

    parser.add_argument('--no-vis', action='store_true', help='Disable visualization window and only output statistics')
    parser.add_argument('--save-debug', action='store_true', help='Save a visualization match image to the disk')
    args = parser.parse_args()

    # Parse records to get the total count for boundary checks
    records = parse_csv(args.csv_path)
    total_records = len(records)
    if not records:
        print('Error: No valid records found in overview CSV.')
        sys.exit(1)

    if args.no_vis:
        evaluate_pair(
            csv_path=args.csv_path,
            index=args.index,
            config=args.config,
            road_width=args.road_width,
            lane_width_fraction=args.lane_width_fraction,
            near=args.near,
            resolution=args.resolution,
            tolerance=args.tolerance,
            no_vis=True,
            save_debug=args.save_debug,
        )
    else:
        index = args.index
        win_name = 'BEV Alignment Validator (Consecutive Frame Pair)'
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

        while True:
            vis_img, _ = evaluate_pair(
                csv_path=args.csv_path,
                index=index,
                config=args.config,
                road_width=args.road_width,
                lane_width_fraction=args.lane_width_fraction,
                near=args.near,
                resolution=args.resolution,
                tolerance=args.tolerance,
                no_vis=True,  # We handle display in the loop
                save_debug=args.save_debug,
            )

            if vis_img is None:
                print(f'Error: Failed to evaluate pair for index {index}')
                break

            cv2.imshow(win_name, vis_img)
            print(
                f'\n[Pair {index} shown] Controls: Right Arrow/d/n for Next, Left Arrow/a/p for Prev, ESC/q to Quit...'
            )

            key = cv2.waitKeyEx(0)

            # ESC or standard ASCII 'q'/'Q' (not extended keys)
            if key == 27 or (key < 256 and key in [ord('q'), ord('Q')]):
                break
            # Right Arrow, 'd', or 'n' -> Next index
            elif key == 2555904 or key == 65363 or (key & 0xFF) in [ord('d'), ord('D'), ord('n'), ord('N')]:
                if index < total_records - 3:
                    index += 1
                else:
                    print('Already at the last consecutive pair.')
            # Left Arrow, 'a', or 'p' -> Prev index
            elif key == 2424832 or key == 65361 or (key & 0xFF) in [ord('a'), ord('A'), ord('p'), ord('P')]:
                if index > 0:
                    index -= 1
                else:
                    print('Already at the first consecutive pair.')

        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
