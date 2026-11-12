"""Fit an exploratory floor homography from five confirmed tile-corner clicks.

Order: near centre, next centre, far centre, left, right. The last two are
corners of the square tile between the first two. Coordinates are relative to
click 1, forward along 1->2, left positive. This is not a robot calibration.
"""

import argparse
import base64
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np


def transform(points, matrix):
    return cv2.perspectiveTransform(np.asarray(points, dtype=float)[None], matrix)[0]


def fit(points, ground):
    # Manual click uncertainty is in pixels: minimize reprojection in image space.
    to_image, _ = cv2.findHomography(ground, points, method=0)
    if to_image is None or not np.all(np.isfinite(to_image)) or np.linalg.matrix_rank(to_image) != 3:
        raise ValueError('Tile points do not define an invertible projection')
    to_floor = np.linalg.inv(to_image)
    to_floor /= np.linalg.norm(to_floor)
    return to_floor, to_image


def fit_tile_projection(annotations, click_numbers=(1, 2, 3, 4, 5), tile_size=0.3):
    if not math.isfinite(tile_size) or tile_size <= 0:
        raise ValueError('Tile size must be finite and positive')
    if len(click_numbers) != 5 or len(set(click_numbers)) != 5 or min(click_numbers) < 1:
        raise ValueError('Select five distinct one-based click numbers')
    if max(click_numbers) > len(annotations['clicks']):
        raise ValueError('Selected click number does not exist')
    clicks = [annotations['clicks'][i - 1] for i in click_numbers]
    frames = [annotations['frames'][c['frame_id']] for c in clicks]
    sources = {(f['source']['file'], f['source']['recording_start_utc']) for f in frames}
    sizes = {tuple(f['source_size']) for f in frames}
    if len(sources) != 1 or len(sizes) != 1:
        raise ValueError('All five clicks must share a recording and source resolution')
    size = np.asarray(frames[0]['source_size'], dtype=float)
    points = np.asarray([c['pixel_xy'] for c in clicks], dtype=float)
    if (
        size.shape != (2,)
        or not np.all(np.isfinite(size))
        or np.any(size <= 0)
        or points.shape != (5, 2)
        or not np.all(np.isfinite(points))
        or np.any(points < 0)
        or np.any(points >= size)
    ):
        raise ValueError('Pixels must be finite and inside the source image')
    half_diagonal = tile_size / math.sqrt(2)
    ground = np.array(
        [
            [0, 0],
            [2 * half_diagonal, 0],
            [4 * half_diagonal, 0],
            [half_diagonal, half_diagonal],
            [half_diagonal, -half_diagonal],
        ]
    )
    to_floor, to_image = fit(points, ground)
    predicted = transform(points, to_floor)
    projected_pixels = transform(ground, to_image)
    # Independent geometric check: exclude the far centre corner from the near tile fit.
    near = [0, 1, 3, 4]
    near_to_floor, _ = fit(points[near], ground[near])
    held_out = transform(points[[2]], near_to_floor)[0]
    floor_errors = np.linalg.norm(predicted - ground, axis=1)
    pixel_errors = np.linalg.norm(projected_pixels - points, axis=1)
    return dict(
        schema='click2go-exploratory-tile-fit-v1',
        coordinate_frame='click_1_origin_forward_to_click_2_left_positive',
        tile_side_m=tile_size,
        source_size=size.tolist(),
        assumptions=[
            'Confirmed square tile corner identities',
            'Stationary camera across the selected snapshots',
            'Flat floor',
            'Raw image fit without lens undistortion',
        ],
        limitations=[
            'No measured robot origin or camera mounting pose',
            'Fit residuals are not independent validation',
            'Do not use this file as live_monitor --calibration or for robot control',
        ],
        image_to_floor=to_floor.tolist(),
        floor_to_image=to_image.tolist(),
        fit_rms_m=float(np.sqrt(np.mean(floor_errors**2))),
        fit_rms_px=float(np.sqrt(np.mean(pixel_errors**2))),
        points=[
            dict(
                click_number=i,
                source=f['source'],
                pixel_xy=p.tolist(),
                expected_floor_xy_m=q.tolist(),
                fitted_floor_xy_m=r.tolist(),
                fitted_pixel_xy=pp.tolist(),
                error_m=float(em),
                error_px=float(ep),
            )
            for i, f, p, q, r, pp, em, ep in zip(
                click_numbers, frames, points, ground, predicted, projected_pixels, floor_errors, pixel_errors
            )
        ],
        held_out_check=dict(
            fitted_click_numbers=[click_numbers[i] for i in near],
            excluded_click_number=click_numbers[2],
            expected_floor_xy_m=ground[2].tolist(),
            predicted_floor_xy_m=held_out.tolist(),
            error_m=float(np.linalg.norm(held_out - ground[2])),
        ),
    )


def plot_fit(annotations, report, output):
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    first = annotations['clicks'][report['points'][0]['click_number'] - 1]
    frame = annotations['frames'][first['frame_id']]
    jpeg = base64.b64decode(frame['jpeg_data_url'].split(',', 1)[1])
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    width, height = report['source_size']
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    axes[0].imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), extent=(0, width, height, 0))
    axes[0].set_title('Clicks across snapshots, shown on first image\nAssumes stationary camera')
    expected = np.array([p['expected_floor_xy_m'] for p in report['points']])
    predicted = np.array([p['fitted_floor_xy_m'] for p in report['points']])
    to_image = np.array(report['floor_to_image'])
    # Overlay only the confirmed tile and the next centre point, not a distant extrapolated grid.
    for edge in ([0, 3, 1, 4, 0], [1, 2]):
        pixels = transform(expected[edge], to_image)
        axes[0].plot(pixels[:, 0], pixels[:, 1], '--', color='cyan', linewidth=1)
    for point in report['points']:
        x, y = point['pixel_xy']
        axes[0].plot(x, y, '+', color='yellow', markersize=12)
        axes[0].annotate(
            str(point['click_number']),
            (x, y),
            xytext=(5, -10),
            textcoords='offset points',
            color='yellow',
            bbox=dict(facecolor='black', alpha=0.7),
        )
    axes[0].set_xlim(0, width)
    axes[0].set_ylim(height, 0)
    axes[0].set_xlabel('Source pixel x')
    axes[0].set_ylabel('Source pixel y')
    ax = axes[1]
    ax.plot(expected[:, 1], expected[:, 0], 'o', label='Confirmed tile geometry')
    ax.plot(predicted[:, 1], predicted[:, 0], '+', markersize=10, label='Five-click fit')
    for i, (q, r) in enumerate(zip(expected, predicted)):
        ax.plot([q[1], r[1]], [q[0], r[0]], color='grey')
        ax.annotate(str(report['points'][i]['click_number']), (q[1], q[0]), xytext=(7, 0), textcoords='offset points')
    check = report['held_out_check']
    hx, hy = check['predicted_floor_xy_m']
    ax.plot(hy, hx, 'x', markersize=10, color='red', label='Click 3 predicted from near tile only')
    ax.plot([hy, expected[2, 1]], [hx, expected[2, 0]], '--', color='red')
    ax.set_title(
        f'Fit RMS {report["fit_rms_m"] * 100:.2f} cm / {report["fit_rms_px"]:.2f} px\n'
        f'Held-out click 3 error {check["error_m"] * 100:.1f} cm'
    )
    ax.set_xlabel('Left of click 1 (m)')
    ax.set_ylabel('Forward from click 1 (m)')
    ax.invert_xaxis()
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc='upper center', bbox_to_anchor=(0.5, -0.18))
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('annotations', type=Path)
    parser.add_argument('--clicks', type=int, nargs=5, default=[1, 2, 3, 4, 5])
    parser.add_argument('--tile-size', type=float, default=0.3, help='Square tile side in metres')
    parser.add_argument('--output', type=Path, required=True, help='Output directory for fit.json and projection.png')
    args = parser.parse_args()
    content = args.annotations.read_bytes()
    annotations = json.loads(content)
    report = fit_tile_projection(annotations, args.clicks, args.tile_size)
    report['annotation_file'] = args.annotations.name
    report['annotation_sha256'] = hashlib.sha256(content).hexdigest()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'fit.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    plot_fit(annotations, report, args.output / 'projection.png')
    print(f'Fit RMS: {report["fit_rms_m"] * 100:.2f} cm; {report["fit_rms_px"]:.2f} source pixels')
    print(f'Held-out click 3 error: {report["held_out_check"]["error_m"] * 100:.1f} cm')
    print(f'Report: {args.output}')


if __name__ == '__main__':
    main()
