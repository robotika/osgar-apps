import unittest
from datetime import timedelta
from unittest.mock import patch

import numpy as np

from lidarroad import (
    analyze_scan,
    batch_processing,
    calculate_road_width,
    draw_batch,
    draw_scan,
    get_best_match,
    slow_get_best_match,
)


class TestLidarRoad(unittest.TestCase):
    def test_get_best_match(self):
        # Basic validation case
        mask = [0, 0, 1, 1, 1, 0, 0]
        from_i, to_i = get_best_match(mask, 3)
        self.assertEqual(from_i, 2)
        self.assertEqual(to_i, 5)

        # Bug reproduction: optimal window is at the very end of the array
        mask_end = [0, 0, 1, 1, 1]
        from_i_end, to_i_end = get_best_match(mask_end, 3)
        self.assertEqual(from_i_end, 2)
        self.assertEqual(to_i_end, 5)

        from_i_slow, to_i_slow = slow_get_best_match(mask_end, 3)
        self.assertEqual(from_i_slow, 2)
        self.assertEqual(to_i_slow, 5)

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
                for prev_from_i, penalty_weight in [(None, 0.0), (10, 0.2), (50, 0.5), (0, 1.0)]:
                    from_fast, to_fast = get_best_match(
                        m, window_size, prev_from_i=prev_from_i, penalty_weight=penalty_weight
                    )
                    from_slow, to_slow = slow_get_best_match(
                        m, window_size, prev_from_i=prev_from_i, penalty_weight=penalty_weight
                    )
                    msg = (
                        f"Mismatch for window_size {window_size} on "
                        f"mask {m if len(m) < 20 else 'random'} with "
                        f"prev_from_i={prev_from_i}, weight={penalty_weight}"
                    )
                    self.assertEqual((from_fast, to_fast), (from_slow, to_slow), msg)

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

    @patch('matplotlib.pyplot.show')
    @patch('matplotlib.pyplot.legend')
    @patch('matplotlib.pyplot.ylabel')
    @patch('matplotlib.pyplot.xlabel')
    @patch('matplotlib.pyplot.title')
    @patch('matplotlib.pyplot.plot')
    def test_draw_batch(self, mock_plot, mock_title, mock_xlabel, mock_ylabel, mock_legend, mock_show):
        times = [1.0, 2.0, 3.0]
        from_i = [100, 110, 120]
        to_i = [400, 410, 420]
        draw_batch(times, from_i, to_i)

        self.assertEqual(mock_plot.call_count, 2)
        mock_plot.assert_any_call(times, from_i, 'g.-', label='from_i')
        mock_plot.assert_any_call(times, to_i, 'b.-', label='to_i')

        mock_xlabel.assert_called_once_with('Time (s)')
        mock_ylabel.assert_called_once_with('Scan Index')
        mock_title.assert_called_once_with('Best Matching Interval Boundaries Over Time')
        mock_legend.assert_called_once()
        mock_show.assert_called_once()

    def test_batch_processing(self):
        mock_scan = [10] * 1800
        log = [
            (timedelta(seconds=1.0), 'vanjee.scan10', mock_scan),
            (timedelta(seconds=2.0), 'vanjee.scan10', mock_scan),
            (timedelta(seconds=3.0), 'vanjee.scan10', mock_scan),
            (timedelta(seconds=4.0), 'vanjee.scan10', mock_scan)
        ]

        # Test standard batch processing (slicing=False by default)
        times, from_indices, to_indices = batch_processing(
            log, start_sec=1.5, end_sec=3.5, tolerance=10, window_size=500, fast=False
        )
        self.assertEqual(times, [2.0, 3.0])
        self.assertEqual(from_indices, [0, 0])
        self.assertEqual(to_indices, [500, 500])

        # Test fast batch processing
        times_f, from_indices_f, to_indices_f = batch_processing(
            log, start_sec=1.5, end_sec=3.5, tolerance=10, window_size=500, fast=True
        )
        self.assertEqual(times_f, [2.0, 3.0])
        self.assertEqual(from_indices_f, [0, 0])
        self.assertEqual(to_indices_f, [500, 500])

        # Test batch processing with slicing=True
        times_s, from_indices_s, to_indices_s = batch_processing(
            log, start_sec=1.5, end_sec=3.5, tolerance=10, window_size=500, fast=False, slicing=True
        )
        self.assertEqual(times_s, [2.0, 3.0])
        self.assertEqual(from_indices_s, [0, 0])
        self.assertEqual(to_indices_s, [500, 500])

    @patch('lidarroad.lidarroad.slow_get_best_match')
    def test_analyze_scan(self, mock_slow):
        mock_slow.return_value = (0, 500)
        scan = [10] * 1800

        # Standard mode (runs and asserts slow version matches)
        diff, from_i, to_i = analyze_scan(scan, tolerance=10, window_size=500, fast=False)
        np.testing.assert_array_equal(diff, np.zeros(1799))
        self.assertEqual(from_i, 0)
        self.assertEqual(to_i, 500)
        mock_slow.assert_called_once()

        mock_slow.reset_mock()

        # Fast mode (skips slow match calculation and assertion)
        diff_fast, from_i_fast, to_i_fast = analyze_scan(scan, tolerance=10, window_size=500, fast=True)
        np.testing.assert_array_equal(diff_fast, np.zeros(1799))
        self.assertEqual(from_i_fast, 0)
        self.assertEqual(to_i_fast, 500)
        mock_slow.assert_not_called()

    @patch('matplotlib.pyplot.subplots')
    @patch('matplotlib.pyplot.show')
    def test_draw_scan_with_original(self, mock_show, mock_subplots):
        from unittest.mock import MagicMock

        mock_ax1 = MagicMock()
        mock_ax2 = MagicMock()
        mock_ax3 = MagicMock()
        mock_subplots.return_value = (MagicMock(), (mock_ax1, mock_ax2, mock_ax3))

        scan = [1, 2, 3]
        original = [10, 11, 12, 13]
        draw_scan(scan, tolerance=15, interval=(1, 2), original_scan=original)

        mock_subplots.assert_called_once_with(1, 3, sharex=True)
        mock_ax1.plot.assert_called_once_with(original, label='Scan')
        mock_ax2.plot.assert_called_once_with(scan)

        # Check boundary/tolerance overlays
        mock_ax1.axvline.assert_any_call(x=1, color='g', linestyle='--', label='Tracked Match')
        mock_ax1.axvline.assert_any_call(x=2, color='g', linestyle='--')

        mock_ax2.axhline.assert_any_call(y=15, color='r', linestyle='--')
        mock_ax2.axhline.assert_any_call(y=-15, color='r', linestyle='--')
        mock_ax2.axvline.assert_any_call(x=1, color='g', linestyle='--', label='Tracked Match')
        mock_ax2.axvline.assert_any_call(x=2, color='g', linestyle='--')

        # Check ax3 cost function plot
        self.assertEqual(mock_ax3.plot.call_count, 3)  # plot window_sums, global argmax, and tracked argmax
        mock_ax3.axvline.assert_any_call(x=1, color='g', linestyle='--')
        mock_show.assert_called_once()

    def test_calculate_road_width(self):
        scan = [1500] * 1800
        width_sliced = calculate_road_width(scan, from_i=0, to_i=900, tilt_deg=10.0, is_sliced=True)
        self.assertAlmostEqual(width_sliced, 2.954423259)

        width_unsliced = calculate_road_width(scan, from_i=450, to_i=1350, tilt_deg=10.0, is_sliced=False)
        self.assertAlmostEqual(width_unsliced, 2.954423259)


if __name__ == '__main__':
    unittest.main()
