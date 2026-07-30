import unittest
from unittest.mock import patch
import numpy as np

from lidarroad import get_best_match, slow_get_best_match, analyze_scan, draw_scan

class TestLidarRoad(unittest.TestCase):
    def test_get_best_match(self):
        # Basic validation case
        mask = [0, 0, 1, 1, 1, 0, 0]
        from_i, to_i = get_best_match(mask, 3)
        self.assertEqual(from_i, 2)
        self.assertEqual(to_i, 5)

        # Extended comparative validation between fast and slow implementations
        np.random.seed(42)  # For deterministic reproducibility
        masks = [
            [0, 0, 1, 1, 1, 0, 0],
            [1, 1, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 0, 0, 0],
            [1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1],
            np.random.randint(0, 2, 1000).tolist()
        ]
        for m in masks:
            for window_size in [1, 3, 5, 50, 100]:
                if len(m) > window_size:
                    from_fast, to_fast = get_best_match(m, window_size)
                    from_slow, to_slow = slow_get_best_match(m, window_size)
                    self.assertEqual(
                        (from_fast, to_fast), (from_slow, to_slow),
                        f"Mismatch for window_size {window_size} on mask {m if len(m) < 20 else 'random'}"
                    )

    @patch('matplotlib.pyplot.show')
    @patch('matplotlib.pyplot.axvline')
    @patch('matplotlib.pyplot.axhline')
    @patch('matplotlib.pyplot.plot')
    def test_draw_scan_with_tolerance_and_interval(self, mock_plot, mock_axhline, mock_axvline, mock_show):
        scan = [1, 2, 3]
        draw_scan(scan, tolerance=15, interval=(10, 20))
        mock_plot.assert_called_once_with(scan)
        
        # Check horizontal lines
        self.assertEqual(mock_axhline.call_count, 2)
        mock_axhline.assert_any_call(y=15, color='r', linestyle='--')
        mock_axhline.assert_any_call(y=-15, color='r', linestyle='--')
        
        # Check vertical lines
        self.assertEqual(mock_axvline.call_count, 2)
        mock_axvline.assert_any_call(x=10, color='g', linestyle='--')
        mock_axvline.assert_any_call(x=20, color='g', linestyle='--')
        
        mock_show.assert_called_once()

    @patch('matplotlib.pyplot.show')
    @patch('matplotlib.pyplot.axvline')
    @patch('matplotlib.pyplot.axhline')
    @patch('matplotlib.pyplot.plot')
    def test_draw_scan_without_tolerance_and_interval(self, mock_plot, mock_axhline, mock_axvline, mock_show):
        scan = [1, 2, 3]
        draw_scan(scan, tolerance=None, interval=None)
        mock_plot.assert_called_once_with(scan)
        mock_axhline.assert_not_called()
        mock_axvline.assert_not_called()
        mock_show.assert_called_once()

    def test_analyze_scan(self):
        # scan of length 1800
        scan = [10] * 1800
        diff, from_i, to_i = analyze_scan(scan, tolerance=10, window_size=500)
        # Expected diff of [10] * 1800 is an array of 1799 zeros
        np.testing.assert_array_equal(diff, np.zeros(1799))
        self.assertEqual(from_i, 0)
        self.assertEqual(to_i, 500)


if __name__ == '__main__':
    unittest.main()
