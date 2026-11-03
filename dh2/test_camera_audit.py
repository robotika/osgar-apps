import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from dh2.camera_audit import calibration_geometry, clock_variation, quantization_check


class CameraAuditTest(unittest.TestCase):
    def test_clock_origins_do_not_turn_relative_variation_into_latency(self):
        receipts = np.array([10., 10.12, 10.21])
        acquisitions = np.array([1000., 1000.1, 1000.2])
        first = clock_variation(receipts, acquisitions)
        shifted = clock_variation(receipts, acquisitions + 300)
        self.assertAlmostEqual(first['excess_above_minimum_s']['maximum'], .02)
        self.assertAlmostEqual(first['excess_above_minimum_s']['maximum'],
                               shifted['excess_above_minimum_s']['maximum'])
        self.assertNotEqual(first['receipt_minus_acquisition_s']['minimum'],
                            shifted['receipt_minus_acquisition_s']['minimum'])
        with self.assertRaises(ValueError):
            clock_variation(receipts, [1000, 1000.1, 0])

    def test_quantized_depth_and_wrong_conversion_scale(self):
        histogram = np.zeros(3001, dtype=int)
        histogram[2506], histogram[1671] = 100, 200
        result = quantization_check(histogram, 30.073274)
        self.assertEqual(result['sampled_points'], 300)
        self.assertEqual(result['fraction_within_1mm_integer_disparity'], 1)
        wrong = quantization_check(histogram, 33)
        self.assertEqual(wrong['fraction_within_1mm_integer_disparity'], 0)
        self.assertEqual(quantization_check(np.zeros(3001, dtype=int), 30)['sampled_points'], 0)

    def test_metric_calibration_and_resizing_do_not_claim_rectified_intrinsics(self):
        camera = dict(width=1280, height=800, intrinsicMatrix=[[800, 0, 640], [0, 800, 400], [0, 0, 1]],
                      extrinsics=dict(toCameraSocket=2, lengthUnit=1, translation=dict(x=-7.5, y=0, z=0)))
        calibration = dict(device_id='test', depthai_version='test', eeprom=dict(
            stereoRectificationData=dict(leftCameraSocket=1, rightCameraSocket=2), cameraData=[[1, camera]]))
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'calibration.json'
            path.write_text(json.dumps(calibration))
            result = calibration_geometry(path, (400, 640))
            self.assertAlmostEqual(result['baseline_m'], .075)
            self.assertAlmostEqual(result['focal_baseline_m_pixel'], 30)
            self.assertFalse(result['verified_rectified_projection'])
            with self.assertRaises(ValueError):
                calibration_geometry(path, (400, 800))


if __name__ == '__main__':
    unittest.main()
