import unittest
from unittest.mock import patch
import numpy as np

from lidarroad import get_best_match, analyze_scan, draw_scan

class TestLidarRoad(unittest.TestCase):
    def test_get_best_match(self):
        mask = [0, 0, 1, 1, 1, 0, 0]
        from_i, to_i = get_best_match(mask, 3)
        self.assertEqual(from_i, 2)
        self.assertEqual(to_i, 5)

    @patch('matplotlib.pyplot.show')
    @patch('matplotlib.pyplot.axhline')
    @patch('matplotlib.pyplot.plot')
    def test_draw_scan_with_tolerance(self, mock_plot, mock_axhline, mock_show):
        scan = [1, 2, 3]
        draw_scan(scan, tolerance=15)
        mock_plot.assert_called_once_with(scan)
        # Check that plt.axhline was called with tolerance and -tolerance
        self.assertEqual(mock_axhline.call_count, 2)
        mock_axhline.assert_any_call(y=15, color='r', linestyle='--')
        mock_axhline.assert_any_call(y=-15, color='r', linestyle='--')
        mock_show.assert_called_once()

    @patch('matplotlib.pyplot.show')
    @patch('matplotlib.pyplot.axhline')
    @patch('matplotlib.pyplot.plot')
    def test_draw_scan_without_tolerance(self, mock_plot, mock_axhline, mock_show):
        scan = [1, 2, 3]
        draw_scan(scan, tolerance=None)
        mock_plot.assert_called_once_with(scan)
        mock_axhline.assert_not_called()
        mock_show.assert_called_once()

    @patch('lidarroad.draw_scan')
    def test_analyze_scan(self, mock_draw_scan):
        # scan of length 1800
        scan = [10] * 1800
        analyze_scan(scan, tolerance=10, window_size=500)
        # Expected diff of [10] * 1800 is an array of 1799 zeros
        mock_draw_scan.assert_called_once()
        called_args, called_kwargs = mock_draw_scan.call_args
        np.testing.assert_array_equal(called_args[0], np.zeros(1799))
        self.assertEqual(called_args[1], 10)


if __name__ == '__main__':
    unittest.main()
