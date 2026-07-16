import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from mosaic import parse_csv, transform_point


class TestMosaic(unittest.TestCase):
    def test_transform_point_origin_no_rotation(self):
        # Robot at (0, 0) facing 0 radians
        gx, gy = transform_point(x_local=2.0, y_local=1.0, robot_x=0.0, robot_y=0.0, heading=0.0)
        self.assertAlmostEqual(gx, 2.0)
        self.assertAlmostEqual(gy, 1.0)

    def test_transform_point_with_translation_and_rotation(self):
        # Robot at (5.0, 10.0) facing pi/2 (90 degrees, pointing along +Y)
        # local forward point X_local=3.0, Y_local=0.0 should be at global X = 5.0, Y = 13.0
        gx, gy = transform_point(x_local=3.0, y_local=0.0, robot_x=5.0, robot_y=10.0, heading=math.pi / 2.0)
        self.assertAlmostEqual(gx, 5.0)
        self.assertAlmostEqual(gy, 13.0)

        # local left point X_local=0.0, Y_local=2.0 (left of robot) should be at global X=3.0, Y=10.0 (pointing west)
        gx, gy = transform_point(x_local=0.0, y_local=2.0, robot_x=5.0, robot_y=10.0, heading=math.pi / 2.0)
        self.assertAlmostEqual(gx, 3.0)
        self.assertAlmostEqual(gy, 10.0)

    def test_parse_csv_valid_and_invalid(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = os.path.join(tmpdir, "overview.csv")
            with open(csv_path, "w", encoding="utf-8") as f:
                f.write("img-0000.jpg,1.5,10.0,20.0,1.570796\n")
                f.write("invalid_line,1.2\n") # Too short
                f.write("img-0001.jpg,abc,30.0,40.0,0.0\n") # Invalid float
                f.write("img-0002.jpg,3.5,30.0,40.0,0.0\n")

            records = parse_csv(csv_path)
            self.assertEqual(len(records), 2)
            self.assertEqual(records[0]['filename'], "img-0000.jpg")
            self.assertEqual(records[0]['ts'], 1.5)
            self.assertEqual(records[0]['x'], 10.0)
            self.assertEqual(records[0]['y'], 20.0)
            self.assertAlmostEqual(records[0]['heading'], 1.570796)

            self.assertEqual(records[1]['filename'], "img-0002.jpg")
            self.assertEqual(records[1]['ts'], 3.5)
            self.assertEqual(records[1]['x'], 30.0)
            self.assertEqual(records[1]['y'], 40.0)
            self.assertAlmostEqual(records[1]['heading'], 0.0)


if __name__ == "__main__":
    unittest.main()
