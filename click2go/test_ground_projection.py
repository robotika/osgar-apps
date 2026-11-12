"""Geometry checks with synthetic calibration; no claim about M04 calibration."""

import unittest

import cv2
import numpy as np
from ground_projection import GroundProjection


def synthetic_calibration():
    return dict(
        image_size=[1920, 1080],
        camera_matrix=[[1000, 0, 960], [0, 1000, 540], [0, 0, 1]],
        distortion_coefficients=[0, 0, 0, 0, 0],
        camera_to_robot_rotation=[[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
        camera_position_m=[0.1, 0, 0.5],
        max_range_m=5,
    )


class ProjectionTest(unittest.TestCase):
    def test_ground_point_and_left_right_sign(self):
        projection = GroundProjection(synthetic_calibration())
        np.testing.assert_allclose(projection.project(960, 1040, [1920, 1080]), [1.1, 0, 0])
        np.testing.assert_allclose(projection.project(460, 1040, [1920, 1080]), [1.1, 0.5, 0])
        np.testing.assert_allclose(projection.project(1460, 1040, [1920, 1080]), [1.1, -0.5, 0])

    def test_reject_horizon_sky_far_pixels_and_wrong_size(self):
        projection = GroundProjection(synthetic_calibration())
        for x, y, size in (
            (960, 540, [1920, 1080]),
            (960, 100, [1920, 1080]),
            (960, 541, [1920, 1080]),
            (-1, 800, [1920, 1080]),
            (1920, 800, [1920, 1080]),
            (960, float('nan'), [1920, 1080]),
            (960, 800, [640, 360]),
            (960, 800, None),
        ):
            with self.subTest(x=x, y=y, size=size), self.assertRaises(ValueError):
                projection.project(x, y, size)

    def test_distorted_and_tilted_camera_round_trip(self):
        calibration = synthetic_calibration()
        tilt, _ = cv2.Rodrigues(np.array([0.0, 0.15, 0.0]))
        rotation = tilt @ np.asarray(calibration['camera_to_robot_rotation'])
        calibration['camera_to_robot_rotation'] = rotation.tolist()
        calibration['distortion_coefficients'] = [0.1, -0.02, 0.001, -0.002, 0.0]
        target = np.array([1.5, 0.2, 0.0])
        optical = rotation.T @ (target - np.array(calibration['camera_position_m']))
        pixel, _ = cv2.projectPoints(
            optical.reshape(1, 3),
            np.zeros(3),
            np.zeros(3),
            np.array(calibration['camera_matrix'], dtype=float),
            np.array(calibration['distortion_coefficients']),
        )
        x, y = pixel[0, 0]
        np.testing.assert_allclose(GroundProjection(calibration).project(x, y, [1920, 1080]), target, atol=1e-5)

    def test_reject_invalid_calibration(self):
        for key, value in (
            ('camera_position_m', [0, 0, 0]),
            ('camera_to_robot_rotation', [[1, 0, 0], [0, 1, 0], [0, 0, -1]]),
            ('camera_matrix', [[-1, 0, 960], [0, 1000, 540], [0, 0, 1]]),
            ('distortion_coefficients', [float('nan')] * 5),
            ('max_range_m', float('inf')),
        ):
            calibration = synthetic_calibration()
            calibration[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                GroundProjection(calibration)
