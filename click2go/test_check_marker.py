"""Physical marker checks with known synthetic optics and mounting geometry."""

import math
import unittest

import numpy as np
from check_marker import check_marker, level_camera_calibration


def synthetic_data():
    k = [[1000, 0, 960], [0, 1000, 540], [0, 0, 1]]
    factory = dict(
        device_id='synthetic',
        eeprom=dict(cameraData=[[0, dict(width=1920, height=1080, intrinsicMatrix=k, distortionCoeff=[0] * 5)]]),
    )
    a = 0.3 / math.sqrt(2)
    tile = np.array([[0, 0], [2 * a, 0], [4 * a, 0], [a, a], [a, -a], [0, 0]])
    robot = tile + [1.05, 0]
    pixels = np.column_stack(
        (960 - 1000 * robot[:, 1] / (robot[:, 0] - 0.05), 540 + 1000 * 0.29 / (robot[:, 0] - 0.05))
    )
    frame = dict(source=dict(file='tile.log', recording_start_utc=100.0), source_size=[1920, 1080])
    annotations = dict(frames={'tile': frame}, clicks=[dict(frame_id='tile', pixel_xy=p.tolist()) for p in pixels])
    return annotations, factory


class MarkerCheckTest(unittest.TestCase):
    def test_recovers_known_camera_distance_and_axle_reference(self):
        annotations, factory = synthetic_data()
        report = check_marker(annotations, factory, 6, 1.0)
        np.testing.assert_allclose(report['marker_projected_from_axle_m'], [1.05, 0, 0], atol=1e-10)
        np.testing.assert_allclose(report['tile_origin_from_axle_xy_m'], [1.05, 0], atol=1e-6)
        self.assertLess(abs(report['marker_forward_error_m']), 1e-10)
        self.assertTrue(all(abs(p['error_m']) < 1e-10 for p in report['tile_pair_checks']))

    def test_scales_factory_intrinsics_without_changing_distortion(self):
        _, factory = synthetic_data()
        calibration = level_camera_calibration(factory, [960, 540], 0.29, 0.05)
        self.assertEqual(calibration['camera_matrix'], [[500, 0, 480], [0, 500, 270], [0, 0, 1]])
        self.assertEqual(calibration['distortion_coefficients'], [0] * 5)
        with self.assertRaises(ValueError):
            level_camera_calibration(factory, [640, 400], 0.29, 0.05)

    def test_reject_invalid_measurements_and_marker_selection(self):
        annotations, factory = synthetic_data()
        for index, distance, height in (
            (5, 1.0, 0.29),
            (7, 1.0, 0.29),
            (6, -1.0, 0.29),
            (6, float('nan'), 0.29),
            (6, 1.0, 0),
        ):
            with self.subTest(index=index, distance=distance, height=height), self.assertRaises(ValueError):
                check_marker(annotations, factory, index, distance, height)
