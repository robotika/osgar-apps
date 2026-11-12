"""Check a measured floor marker against an approximate level-camera model.

Uses an existing OAK EEPROM RGB calibration export and operator-supplied mounting
measurements. Output is a preview candidate, not a validated motion calibration.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from fit_tile_projection import fit_tile_projection, transform
from ground_projection import GroundProjection


def level_camera_calibration(factory, size, height, forward):
    rgb = dict(factory['eeprom']['cameraData'])[0]  # CAM_A / RGB
    factory_size = np.array([rgb['width'], rgb['height']], dtype=float)
    size = np.array(size, dtype=float)
    if factory_size.shape != (2,) or np.any(factory_size <= 0):
        raise ValueError('Invalid factory image size')
    scale = size / factory_size
    if not np.all(np.isfinite(scale)) or not math.isclose(scale[0], scale[1], rel_tol=1e-6):
        raise ValueError('Expected same-aspect RGB resize; cropping needs separate calibration')
    matrix = np.array(rgb['intrinsicMatrix'], dtype=float)
    matrix[:2] *= scale[0]
    calibration = dict(
        image_size=size.astype(int).tolist(),
        camera_matrix=matrix.tolist(),
        distortion_coefficients=rgb['distortionCoeff'],
        camera_to_robot_rotation=[[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
        camera_position_m=[forward, 0, height],
        max_range_m=2.5,
    )
    # Use the same validation and projection implementation as the live preview.
    return GroundProjection(calibration).calibration


def check_marker(annotations, factory, marker_number, distance, height=0.29, forward=0.05):
    if not math.isfinite(distance) or distance <= 0:
        raise ValueError('Marker distance must be finite and positive')
    if not math.isfinite(forward):
        raise ValueError('Camera forward offset must be finite')
    if marker_number <= 5 or marker_number > len(annotations['clicks']):
        raise ValueError('Marker must be a separate click after the five tile clicks')
    marker = annotations['clicks'][marker_number - 1]
    frame = annotations['frames'][marker['frame_id']]
    tile_report = fit_tile_projection(annotations)
    if frame['source_size'] != tile_report['source_size']:
        raise ValueError('Marker and tile images must have the same source resolution')
    calibration = level_camera_calibration(factory, frame['source_size'], height, forward)
    projection = GroundProjection(calibration)
    marker_robot = np.array(projection.project(*marker['pixel_xy'], frame['source_size']))
    tile_points = np.array(
        [projection.project(*c['pixel_xy'], frame['source_size'])[:2] for c in annotations['clicks'][:5]]
    )
    marker_tile = transform([marker['pixel_xy']], np.array(tile_report['image_to_floor']))[0]
    # One marker anchors translation only; it does not validate tile scaling/rotation.
    tile_to_axle = np.array([forward + distance, 0]) - marker_tile
    expected = np.array([p['expected_floor_xy_m'] for p in tile_report['points']]) + tile_to_axle
    pairs = []
    for a, b, expected_length in (
        (0, 1, 0.3 * math.sqrt(2)),
        (1, 2, 0.3 * math.sqrt(2)),
        (0, 3, 0.3),
        (3, 1, 0.3),
        (1, 4, 0.3),
        (4, 0, 0.3),
    ):
        actual = float(np.linalg.norm(tile_points[a] - tile_points[b]))
        pairs.append(
            dict(
                click_numbers=[a + 1, b + 1],
                expected_m=expected_length,
                projected_m=actual,
                error_m=actual - expected_length,
            )
        )
    return dict(
        schema='click2go-marker-model-check-v1',
        measurement=dict(
            click_number=marker_number,
            horizontal_camera_distance_m=distance,
            camera_height_m=height,
            camera_ahead_of_axle_m=forward,
            source=frame['source'],
        ),
        assumptions=[
            'Same stationary camera pose across recordings',
            'Factory export belongs to this camera and RGB stream is an uncropped resize',
            'Level camera, aligned forward with tile centreline; zero lateral mounting offset',
            'Marker clicked on the floor centreline',
        ],
        factory_device_id=factory['device_id'],
        marker_projected_from_axle_m=marker_robot.tolist(),
        marker_forward_error_m=float(marker_robot[0] - forward - distance),
        marker_lateral_error_m=float(marker_robot[1]),
        marker_floor_relative_xy_m=marker_tile.tolist(),
        tile_origin_from_axle_xy_m=tile_to_axle.tolist(),
        tile_model_points_from_axle_xy_m=tile_points.tolist(),
        tile_anchored_reference_from_axle_xy_m=expected.tolist(),
        tile_pair_checks=pairs,
        limitations=[
            'One marker is not general range validation',
            'Tile comparison still shows substantial discrepancy',
            'Approximate mounting and candidate factory calibration require verification',
            'No actuator or motion test performed',
        ],
        preview_candidate_calibration=calibration,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('annotations', type=Path)
    parser.add_argument('--factory-calibration', type=Path, required=True)
    parser.add_argument('--marker-click', type=int, default=6)
    parser.add_argument('--horizontal-distance', type=float, required=True)
    parser.add_argument('--height', type=float, default=0.29)
    parser.add_argument('--camera-forward', type=float, default=0.05)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    content = args.annotations.read_bytes()
    factory_content = args.factory_calibration.read_bytes()
    report = check_marker(
        json.loads(content),
        json.loads(factory_content),
        args.marker_click,
        args.horizontal_distance,
        args.height,
        args.camera_forward,
    )
    report.update(
        annotation_file=args.annotations.name,
        annotation_sha256=hashlib.sha256(content).hexdigest(),
        factory_calibration_file=args.factory_calibration.name,
        factory_calibration_sha256=hashlib.sha256(factory_content).hexdigest(),
    )
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'marker-check.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    (args.output / 'm04-ground-candidate.json').write_text(
        json.dumps(report['preview_candidate_calibration'], indent=2, allow_nan=False) + '\n'
    )
    print('Marker from axle:', report['marker_projected_from_axle_m'])
    print(
        f'Forward error: {report["marker_forward_error_m"] * 100:.2f} cm; '
        f'lateral offset: {report["marker_lateral_error_m"] * 100:.2f} cm'
    )


if __name__ == '__main__':
    main()
