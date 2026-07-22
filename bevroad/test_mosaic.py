import argparse
import unittest

from bevroad.mosaic import add_common_arguments


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


if __name__ == '__main__':
    unittest.main()
