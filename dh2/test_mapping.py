import math
import unittest

import numpy as np

from dh2.mapping import SurfaceGrid, interpolate, project_depth


class MappingTest(unittest.TestCase):
    def project(self, depth, **overrides):
        settings = dict(height=0.2, pitch=0, forward=0, left=0, joint_scale=0.5, stride=1)
        settings.update(overrides)
        return project_depth(np.asarray(depth), (100, 100, 0, 0), (0, 0, 0), 0, **settings)

    def test_optical_axes_and_zero_depth(self):
        points = self.project([[1000, 0], [1000, 1000]])
        np.testing.assert_allclose(points, [[1, 0, 0.2], [1, 0, 0.19], [1, -0.01, 0.19]])

    def test_downward_pitch_and_height(self):
        points = self.project([[1000]], pitch=math.pi / 2)
        np.testing.assert_allclose(points, [[0, 0, -0.8]], atol=1e-10)

    def test_heading_joint_and_camera_lever_arm(self):
        points = project_depth(np.array([[1000]]), (100, 100, 0, 0), (2, 3, math.pi / 4),
                               math.pi / 2, height=0.2, pitch=0, forward=0.2, left=0,
                               joint_scale=0.5, stride=1)
        np.testing.assert_allclose(points, [[2, 4.2, 0.2]])

    def test_depth_range_filters_invalid_and_far_samples(self):
        points = self.project([[0, np.nan, 100, 1000, 5000]])
        self.assertEqual(points.shape, (1, 3))
        self.assertEqual(points[0, 0], 1)

    def test_interpolation_wrap_and_missing_data(self):
        values = np.unwrap(np.radians([179, -179]))
        self.assertAlmostEqual(interpolate(0.1, [0, 0.2], values, 0.3), math.pi)
        self.assertIsNone(interpolate(-0.1, [0, 0.2], values, 0.3))
        self.assertIsNone(interpolate(0.4, [0, 0.2], values, 0.3))
        self.assertIsNone(interpolate(0.1, [0, 0.2], values, 0.1))

    def test_grid_retains_surface_heights_and_counts_frames_once(self):
        grid = SurfaceGrid(0.1)
        grid.add(np.array([[0.01, 0.01, 0], [0.02, 0.02, 0.2], [-0.01, 0, -0.4]]))
        grid.add(np.array([[0.01, 0.01, 0.1]]))
        arrays = grid.arrays()
        np.testing.assert_array_equal(arrays['cell_indices'], [[-1, 0], [0, 0]])
        np.testing.assert_array_equal(arrays['frames'], [1, 2])
        np.testing.assert_array_equal(arrays['points'], [1, 3])
        np.testing.assert_allclose(arrays['mean_height'], [-0.4, 0.1])
        np.testing.assert_allclose(arrays['min_height'], [-0.4, 0])
        np.testing.assert_allclose(arrays['max_height'], [-0.4, 0.2])


if __name__ == '__main__':
    unittest.main()
