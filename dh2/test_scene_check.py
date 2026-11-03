import unittest

import numpy as np
from osgar.lib import quaternion as quat

from dh2.scene_check import camera_pose, pose_at, projective_errors, smooth_depth


class SceneCheckTest(unittest.TestCase):
    def test_known_forward_motion_and_incorrect_stationary_pose(self):
        source = np.full((40, 60), 2.)
        target = np.full_like(source, 1.9)
        first = np.zeros(3), np.eye(3)
        current = np.array([0, 0, .1]), np.eye(3)
        errors, count = projective_errors(source, target, first, current, [50, 50, 30, 20], stride=1)
        self.assertGreater(len(errors), count * .8)
        np.testing.assert_allclose(errors, 0, atol=1e-12)
        wrong, _ = projective_errors(source, target, first, first, [50, 50, 30, 20])
        np.testing.assert_allclose(wrong, .1)

    def test_rotated_camera_observes_same_plane(self):
        source = np.full((60, 80), 2.)
        rotation = np.asarray(quat.rotation_matrix(quat.from_axis_angle([0, 1, 0], .1)))
        v, u = np.mgrid[:60, :80]
        rays = np.stack(((u - 40) / 80, (v - 30) / 80, np.ones_like(u)), axis=-1)
        target = 2 / (rays @ rotation.T)[:, :, 2]
        first = np.zeros(3), np.eye(3)
        current = np.zeros(3), rotation
        errors, _ = projective_errors(source, target, first, current, [80, 80, 40, 30])
        self.assertLess(np.percentile(abs(errors), 90), .002)
        wrong, _ = projective_errors(source, target, first, first, [80, 80, 40, 30])
        self.assertGreater(np.median(abs(wrong)), .03)

    def test_invalid_depth_edges_and_large_residuals(self):
        depth = np.full((20, 20), 2.)
        depth[:, 10:] = 1
        depth[5, 5] = 0
        mask = smooth_depth(depth, .3, 3, .08)
        self.assertFalse(mask[:, 9:11].any())
        self.assertFalse(mask[4:7, 5].any())
        first = np.zeros(3), np.eye(3)
        errors, _ = projective_errors(np.full((20, 20), 2.), np.ones((20, 20)),
                                      first, first, [20, 20, 10, 10])
        np.testing.assert_allclose(errors, 1)  # No residual clipping hides inconsistent observations.

    def test_pose_time_uses_acquisition_time_and_unknown_motion_is_excluded(self):
        poses = np.array([1., 1.2]), np.array([[0, 0, 0], [.2, 0, 0]]), [quat.identity()] * 2, [True, True]
        position, _ = pose_at(1.1, poses)
        np.testing.assert_allclose(position, [.1, 0, 0])
        self.assertIsNone(pose_at(.9, poses))
        self.assertIsNone(pose_at(1.1, poses, max_gap=.1))
        self.assertIsNone(pose_at(1.1, (*poses[:3], [True, False])))

    def test_optical_axes_and_rotating_offset(self):
        first = camera_pose((np.zeros(3), quat.identity()), [.21, .04, .16])
        np.testing.assert_allclose(first[1] @ [0, 0, 1], [1, 0, 0])
        turned = camera_pose((np.zeros(3), quat.from_axis_angle([0, 0, 1], np.pi / 2)), [.21, .04, .16])
        np.testing.assert_allclose(turned[0], [-.04, .21, .16], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
