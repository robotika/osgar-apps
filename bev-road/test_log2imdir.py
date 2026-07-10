import datetime
import math
import os
import sys
import tempfile
import unittest

import cv2
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from log2imdir import extract_images_from_reader


class TestLog2ImDir(unittest.TestCase):
    def setUp(self):
        self.test_dir = tempfile.TemporaryDirectory()
        # Generate dummy 10x10 JPEG bytes for testing
        dummy_img = np.zeros((10, 10, 3), dtype=np.uint8)
        _, img_encoded = cv2.imencode('.jpg', dummy_img)
        self.dummy_jpeg_bytes = img_encoded.tobytes()

    def tearDown(self):
        self.test_dir.cleanup()

    def test_extract_images_from_reader_basic(self):
        # We simulate a log that travels 5 meters.
        # Targets: start=1.0m, step=2.0m, end=4.0m (should extract at 1.0m and 3.0m)
        camera_stream = "oak.color"
        pose_stream = "platform.pose2d"

        # Build mock stream data
        # Pose points:
        # P0: (0, 0) at t=0
        # P1: (1000, 0) (1.0m) at t=1
        # P2: (2000, 0) (2.0m) at t=2
        # P3: (3000, 0) (3.0m) at t=3
        # P4: (4000, 0) (4.0m) at t=4
        # P5: (5000, 0) (5.0m) at t=5

        reader_data = [
            # Time, stream, payload
            (datetime.timedelta(seconds=0.0), camera_stream, self.dummy_jpeg_bytes),
            (datetime.timedelta(seconds=0.1), pose_stream, [0, 0, 0]),

            (datetime.timedelta(seconds=1.0), camera_stream, self.dummy_jpeg_bytes),
            (datetime.timedelta(seconds=1.1), pose_stream, [1000, 0, 9000]), # 1.0m, 90 deg (1.57 rad)

            (datetime.timedelta(seconds=2.0), camera_stream, self.dummy_jpeg_bytes),
            (datetime.timedelta(seconds=2.1), pose_stream, [2000, 0, 9000]), # 2.0m

            (datetime.timedelta(seconds=3.0), camera_stream, self.dummy_jpeg_bytes),
            (datetime.timedelta(seconds=3.1), pose_stream, [3000, 0, 9000]), # 3.0m

            (datetime.timedelta(seconds=4.0), camera_stream, self.dummy_jpeg_bytes),
            (datetime.timedelta(seconds=4.1), pose_stream, [4000, 0, 9000]), # 4.0m

            (datetime.timedelta(seconds=5.0), camera_stream, self.dummy_jpeg_bytes),
            (datetime.timedelta(seconds=5.1), pose_stream, [5000, 0, 9000])  # 5.0m
        ]

        count = extract_images_from_reader(
            reader=reader_data,
            output_dir=self.test_dir.name,
            start=1.0,
            step=2.0,
            end=4.0,
            camera_stream=camera_stream,
            pose_stream=pose_stream
        )

        # We expect exactly 2 extracted images (img-0000.jpg at 1.0m, img-0001.jpg at 3.0m)
        self.assertEqual(count, 2)

        # Verify images actually exist
        self.assertTrue(os.path.exists(os.path.join(self.test_dir.name, "img-0000.jpg")))
        self.assertTrue(os.path.exists(os.path.join(self.test_dir.name, "img-0001.jpg")))

        # Verify CSV contents
        csv_path = os.path.join(self.test_dir.name, "overview.csv")
        self.assertTrue(os.path.exists(csv_path))
        with open(csv_path, 'r', encoding="utf-8") as f:
            lines = f.readlines()

        self.assertEqual(len(lines), 2)

        # Verify line content format: image, timestamp, x, y, heading
        # Line 0: img-0000.jpg, 1.1s, x=1.0, y=0.0, heading = 90 deg = 1.570796 rad
        line0_parts = lines[0].strip().split(',')
        self.assertEqual(line0_parts[0], "img-0000.jpg")
        self.assertEqual(float(line0_parts[1]), 1.1)
        self.assertEqual(float(line0_parts[2]), 1.0)
        self.assertEqual(float(line0_parts[3]), 0.0)
        self.assertAlmostEqual(float(line0_parts[4]), math.radians(90.0), places=5)

        # Line 1: img-0001.jpg, 3.1s, x=3.0, y=0.0, heading = 1.570796 rad
        line1_parts = lines[1].strip().split(',')
        self.assertEqual(line1_parts[0], "img-0001.jpg")
        self.assertEqual(float(line1_parts[1]), 3.1)
        self.assertEqual(float(line1_parts[2]), 3.0)
        self.assertEqual(float(line1_parts[3]), 0.0)
        self.assertAlmostEqual(float(line1_parts[4]), math.radians(90.0), places=5)


if __name__ == '__main__':
    unittest.main()
