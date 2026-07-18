import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from check_pair import parse_csv, transform_point, global_to_local, predict_pose


class TestCheckPair(unittest.TestCase):
    def test_transform_point_identity(self):
        # No rotation, origin
        gx, gy = transform_point(1.0, 2.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(gx, 1.0)
        self.assertAlmostEqual(gy, 2.0)

    def test_transform_point_rotation_and_translation(self):
        # Robot at (10.0, 5.0) facing 90 degrees (pi/2 radians, pointing in +Y direction)
        # Local forward point x_local=2.0, y_local=0.0 should end up at global X=10.0, Y=7.0
        gx, gy = transform_point(2.0, 0.0, 10.0, 5.0, math.pi / 2.0)
        self.assertAlmostEqual(gx, 10.0)
        self.assertAlmostEqual(gy, 7.0)

    def test_global_to_local_inverse(self):
        # Test various positions and orientations to ensure global_to_local is the true mathematical inverse
        test_cases = [
            (1.0, 2.0, 0.0, 0.0, 0.0),
            (2.0, -1.0, 10.0, 5.0, math.pi / 2.0),
            (-3.5, 4.2, -20.0, 35.0, -math.pi / 4.0),
            (0.0, 0.0, 100.0, -50.0, 2.5)
        ]
        for xl, yl, rx, ry, heading in test_cases:
            gx, gy = transform_point(xl, yl, rx, ry, heading)
            xl_inv, yl_inv = global_to_local(gx, gy, rx, ry, heading)
            self.assertAlmostEqual(xl_inv, xl)
            self.assertAlmostEqual(yl_inv, yl)

    def test_predict_pose(self):
        # Straight line translation
        rec1 = {'x': 10.0, 'y': 5.0, 'heading': 0.1}
        rec2 = {'x': 12.0, 'y': 6.0, 'heading': 0.1}
        pred = predict_pose(rec1, rec2)
        self.assertAlmostEqual(pred['x'], 14.0)
        self.assertAlmostEqual(pred['y'], 7.0)
        self.assertAlmostEqual(pred['heading'], 0.1)

        # Rotation and translation with heading wrapping
        rec1_wrap = {'x': 0.0, 'y': 0.0, 'heading': 3.1}
        rec2_wrap = {'x': 1.0, 'y': -0.5, 'heading': -3.1}
        # Heading wraps from +3.1 to -3.1.
        # Shortest angular delta should be: -3.1 - 3.1 = -6.2 -> wrapped to ~0.0831853
        # Extrapolated heading: -3.1 + 0.0831853 = -3.0168147
        pred_wrap = predict_pose(rec1_wrap, rec2_wrap)
        self.assertAlmostEqual(pred_wrap['x'], 2.0)
        self.assertAlmostEqual(pred_wrap['y'], -1.0)
        expected_heading = (-3.1 + ((-3.1 - 3.1 + math.pi) % (2 * math.pi) - math.pi))
        self.assertAlmostEqual(pred_wrap['heading'], expected_heading)

    def test_parse_csv_valid_data(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = os.path.join(tmpdir, "overview.csv")
            with open(csv_path, "w", encoding="utf-8") as f:
                f.write("img-0000.jpg,12.5,29.86,1.50,-0.0356\n")
                f.write("img-0001.jpg,13.0,30.36,1.48,-0.0567\n")

            records = parse_csv(csv_path)
            self.assertEqual(len(records), 2)
            self.assertEqual(records[0]['filename'], "img-0000.jpg")
            self.assertEqual(records[0]['ts'], 12.5)
            self.assertEqual(records[0]['x'], 29.86)
            self.assertEqual(records[0]['y'], 1.50)
            self.assertAlmostEqual(records[0]['heading'], -0.0356)


if __name__ == '__main__':
    unittest.main()
