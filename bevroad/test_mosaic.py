import argparse
import json
import tempfile
import unittest
from pathlib import Path

from bevroad.mosaic import (
    add_common_arguments,
    load_calibration_data,
    calculate_spatial_parameters,
    get_bev_corners,
)


class TestArgparse(unittest.TestCase):
    def test_add_common_arguments_basic(self):
        parser = argparse.ArgumentParser()
        add_common_arguments(parser)

        args = parser.parse_args([])
        self.assertIsNone(args.config)
        self.assertEqual(args.road_width, 2.0)
        self.assertEqual(args.lane_width_fraction, 0.5)
        self.assertEqual(args.near, 1.0)
        self.assertEqual(args.resolution, 0.02)
        # Ensure tolerance and margin are not added by default
        self.assertFalse(hasattr(args, 'tolerance'))
        self.assertFalse(hasattr(args, 'margin'))

    def test_add_common_arguments_with_options(self):
        parser = argparse.ArgumentParser()
        add_common_arguments(parser, include_tolerance=True, include_margin=True)

        args = parser.parse_args([])
        self.assertEqual(args.tolerance, 15.0)
        self.assertEqual(args.margin, 0.0)


class TestMosaicHelpers(unittest.TestCase):
    def test_calculate_spatial_parameters(self):
        m_per_pixel, bev_w_m, bev_h_m, d_near, d_far = calculate_spatial_parameters(
            road_width=2.0, bev_width=400, lane_width_fraction=0.5, bev_height=600, near=1.0
        )
        self.assertAlmostEqual(m_per_pixel, 0.01)
        self.assertAlmostEqual(bev_w_m, 4.0)
        self.assertAlmostEqual(bev_h_m, 6.0)
        self.assertAlmostEqual(d_near, 1.0)
        self.assertAlmostEqual(d_far, 7.0)

    def test_get_bev_corners(self):
        corners = get_bev_corners(d_near=1.0, d_far=7.0, bev_w_m=4.0)
        expected = [
            (7.0, 2.0),
            (7.0, -2.0),
            (1.0, -2.0),
            (1.0, 2.0),
        ]
        self.assertEqual(len(corners), 4)
        for c, e in zip(corners, expected):
            self.assertAlmostEqual(c[0], e[0])
            self.assertAlmostEqual(c[1], e[1])

    def test_load_calibration_data_valid(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            config_data = {
                "matrix_M": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                "bev_width": 400,
                "bev_height": 600,
            }
            config_path = tmp_path / "bev_config.json"
            with open(config_path, "w") as f:
                json.dump(config_data, f)

            config_path_str = str(config_path)

            # Load specifying config
            calib, resolved_path = load_calibration_data(tmpdir, config_path_str, verbose=False)
            self.assertEqual(calib["bev_width"], 400)
            self.assertEqual(resolved_path, config_path_str)

            # Load automatically searching directory
            calib_auto, resolved_path_auto = load_calibration_data(tmpdir, None, verbose=False)
            self.assertEqual(calib_auto["bev_width"], 400)
            self.assertEqual(resolved_path_auto, config_path_str)

            # Also check support for pathlib.Path as imdir
            calib_pathlib, resolved_path_pathlib = load_calibration_data(tmp_path, None, verbose=False)
            self.assertEqual(calib_pathlib["bev_width"], 400)
            self.assertEqual(resolved_path_pathlib, config_path_str)

    def test_load_calibration_data_missing_fields(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            config_data = {
                "bev_width": 400,
                "bev_height": 600,
            }
            config_path = tmp_path / "bev_config.json"
            with open(config_path, "w") as f:
                json.dump(config_data, f)

            with self.assertRaises(SystemExit):
                load_calibration_data(tmpdir, str(config_path), verbose=False)


if __name__ == '__main__':
    unittest.main()
