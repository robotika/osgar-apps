"""Offline RGB ray intersection with the robot's flat ground plane.

Optical axes: right/down/forward. Robot axes: forward/left/up, metres.
No odometry transform or actuator interface is provided.
"""

import math

import cv2
import numpy as np


class GroundProjection:
    def __init__(self, calibration):
        self.size = np.asarray(calibration['image_size'], dtype=float)
        self.matrix = np.asarray(calibration['camera_matrix'], dtype=float)
        self.distortion = np.asarray(calibration['distortion_coefficients'], dtype=float)
        self.rotation = np.asarray(calibration['camera_to_robot_rotation'], dtype=float)
        self.origin = np.asarray(calibration['camera_position_m'], dtype=float)
        self.max_range = float(calibration['max_range_m'])
        if self.size.shape != (2,) or not np.all(np.isfinite(self.size)) or np.any(self.size <= 0):
            raise ValueError('image_size must contain positive width and height')
        if not np.all(self.size == np.round(self.size)):
            raise ValueError('image_size must contain integers')
        if self.matrix.shape != (3, 3) or not np.all(np.isfinite(self.matrix)):
            raise ValueError('camera_matrix must be a finite 3x3 matrix')
        if (
            self.matrix[0, 0] <= 0
            or self.matrix[1, 1] <= 0
            or not np.allclose(self.matrix[2], [0, 0, 1])
            or self.matrix[0, 1] != 0
            or self.matrix[1, 0] != 0
        ):
            raise ValueError('camera_matrix must use positive focal lengths and zero skew')
        if self.distortion.shape not in ((4,), (5,), (8,), (12,), (14,)):
            raise ValueError('Use OpenCV pinhole distortion coefficients (4/5/8/12/14 values)')
        if not np.all(np.isfinite(self.distortion)):
            raise ValueError('Distortion coefficients must be finite')
        if (
            self.rotation.shape != (3, 3)
            or not np.all(np.isfinite(self.rotation))
            or not np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-6)
            or not math.isclose(np.linalg.det(self.rotation), 1.0, abs_tol=1e-6)
        ):
            raise ValueError('camera_to_robot_rotation must be a proper orthonormal rotation')
        if self.origin.shape != (3,) or not np.all(np.isfinite(self.origin)) or self.origin[2] <= 0:
            raise ValueError('Camera position must be finite and above ground (z > 0)')
        if not math.isfinite(self.max_range) or self.max_range <= 0:
            raise ValueError('max_range_m must be finite and positive')

    def project(self, x, y, source_size):
        if source_size is None or not np.array_equal(source_size, self.size):
            raise ValueError('Calibration image dimensions do not match the source frame')
        if not (math.isfinite(x) and math.isfinite(y) and 0 <= x < self.size[0] and 0 <= y < self.size[1]):
            raise ValueError('Click must be inside the source image')
        normalized = cv2.undistortPoints(np.array([[[x, y]]], dtype=float), self.matrix, self.distortion)
        direction = self.rotation @ np.array([*normalized[0, 0], 1.0])
        if not np.all(np.isfinite(direction)) or direction[2] >= -1e-6:
            raise ValueError('Ray is above or too close to the ground horizon')
        target = self.origin - self.origin[2] / direction[2] * direction
        if not np.all(np.isfinite(target)) or target[0] <= 0:
            raise ValueError('Ground point is behind the robot')
        if np.linalg.norm(target[:2]) > self.max_range:
            raise ValueError('Ground point exceeds the configured preview range')
        return [float(target[0]), float(target[1]), 0.0]
