import unittest
from datetime import timedelta
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
from osgar.lib import quaternion as quat
from osgar.lib.serialize import serialize
from osgar.logger import LogWriter

from dh2.patch_check import fit_plane, measure_patch, patch_points, selected_frames


def plane(z):
    x, y = np.meshgrid(np.linspace(-.2, .2, 15), np.linspace(-.1, .1, 10))
    return np.column_stack((x.ravel(), y.ravel(), np.full(x.size, z)))


class PatchCheckTest(unittest.TestCase):
    def test_plane_fit_retains_nonplanar_samples(self):
        points = plane(1)
        points[-10:, 2] += .15
        center, normal, errors = fit_plane(points)
        self.assertEqual(len(errors), len(points))
        self.assertAlmostEqual(np.linalg.norm(normal), 1)
        self.assertAlmostEqual(np.mean(errors), 0)
        self.assertGreater(np.max(np.abs(errors)), .05)
        np.testing.assert_allclose(center, np.mean(points, axis=0))

    def test_degenerate_patch_is_rejected(self):
        for points in [plane(1)[:20], np.zeros((40, 3)), np.full((40, 3), np.nan)]:
            with self.assertRaises(ValueError):
                fit_plane(points)

    def test_constant_depth_precision_is_not_accuracy(self):
        pose = np.zeros(3), quat.identity()
        _, _, reference = measure_patch(plane(2), pose, None, np.zeros(3))
        rows, diagnostic, _ = measure_patch(plane(1.8), pose, reference, np.zeros(3))
        articulated = rows[1]
        self.assertLess(articulated['local_p90_abs_m'], 1e-12)
        self.assertAlmostEqual(articulated['reference_median_abs_m'], .2)
        self.assertEqual(diagnostic['dominant_depth_fraction'], 1)

    def test_known_forward_motion_compensates_depth_change(self):
        pose = np.zeros(3), quat.identity()
        _, _, reference = measure_patch(plane(2), pose, None, np.zeros(3))
        moved = np.array([.2, 0, 0]), quat.identity()
        rows, _, _ = measure_patch(plane(1.8), moved, reference, np.zeros(3))
        self.assertAlmostEqual(rows[0]['reference_median_abs_m'], .2)
        self.assertLess(rows[1]['reference_p90_abs_m'], 1e-12)

    def test_plane_cannot_observe_parallel_translation(self):
        pose = np.zeros(3), quat.identity()
        _, _, reference = measure_patch(plane(2), pose, None, np.zeros(3))
        moved = np.array([0, .5, .3]), quat.identity()
        rows, _, _ = measure_patch(plane(2), moved, reference, np.zeros(3))
        self.assertLess(rows[1]['reference_p90_abs_m'], 1e-12)

    def test_pixel_roi_and_invalid_depth(self):
        depth = np.ones((30, 40))
        depth[15, 20] = 0
        points, count = patch_points(depth, [10, 10, 30, 20], [20, 20, 20, 15])
        self.assertEqual(count, 200)
        self.assertEqual(len(points), 195)  # Hole and its four neighbours.
        np.testing.assert_allclose(points[:, 2], 1)
        with self.assertRaises(ValueError):
            patch_points(depth, [10, 10, 45, 20], [20, 20, 20, 15])

    def test_annotation_selects_nearest_actual_frame_and_converts_mm(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'depth.log'
            with LogWriter(filename=str(path), dt=timedelta(0)) as log:
                sid = log.register('oak.depth', dt=timedelta(0))
                for t, distance in [(1, 800), (1.04, 900)]:
                    log.write(sid, serialize(np.full((30, 40), distance, dtype=np.uint16)),
                              dt=timedelta(seconds=t))
            selected = selected_frames(path, [1.03])
            actual, depth = selected[1.03]
            self.assertEqual(actual, 1.04)
            np.testing.assert_allclose(depth, .9)
            with self.assertRaises(ValueError):
                selected_frames(path, [1.2])


if __name__ == '__main__':
    unittest.main()
