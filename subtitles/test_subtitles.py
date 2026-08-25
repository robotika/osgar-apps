import os
import tempfile
import unittest

from subtitles.subtitles import (
    apply_offset,
    format_srt_timestamp,
    generate_segments,
    get_log_duration,
    process_log_to_subtitles,
)


class TestSubtitles(unittest.TestCase):

    def test_format_srt_timestamp(self):
        # Basic cases
        self.assertEqual(format_srt_timestamp(0.0), '00:00:00,000')
        self.assertEqual(format_srt_timestamp(1.123), '00:00:01,123')
        self.assertEqual(format_srt_timestamp(61.5), '00:01:01,500')
        self.assertEqual(format_srt_timestamp(3600.001), '01:00:00,001')

        # Negative values clamped to 0.0
        self.assertEqual(format_srt_timestamp(-5.0), '00:00:00,000')

        # Rounding check
        # 1.0009 seconds -> 1001 ms -> 1s 1ms
        self.assertEqual(format_srt_timestamp(1.0009), '00:00:01,001')
        # 0.9999 seconds -> 1000 ms -> 1s 0ms
        self.assertEqual(format_srt_timestamp(0.9999), '00:00:01,000')

    def test_generate_segments(self):
        # Empty
        self.assertEqual(generate_segments([], 10.0), [])

        # Standard transition: first event starts manual, changes to autonomous
        events = [
            (0.5, 'MANUAL'),
            (5.0, 'AUTONOMOUS'),
            (8.0, 'MANUAL'),
        ]
        segments = generate_segments(events, 10.0)
        expected = [
            (0.5, 5.0, 'MANUAL'),
            (5.0, 8.0, 'AUTONOMOUS'),
            (8.0, 10.0, 'MANUAL'),
        ]
        self.assertEqual(segments, expected)

        # Deduplication of identical consecutive states
        events = [
            (0.5, 'MANUAL'),
            (2.0, 'MANUAL'),  # Duplicate
            (5.0, 'AUTONOMOUS'),
            (5.0, 'AUTONOMOUS'),  # Duplicate same time or separate
            (8.0, 'MANUAL'),
        ]
        segments = generate_segments(events, 10.0)
        self.assertEqual(segments, expected)

    def test_apply_offset(self):
        segments = [
            (0.0, 5.0, 'MANUAL'),
            (5.0, 10.0, 'AUTONOMOUS'),
        ]

        # Positive offset
        offset_pos = apply_offset(segments, 5.0)
        expected_pos = [
            (5.0, 10.0, 'MANUAL'),
            (10.0, 15.0, 'AUTONOMOUS'),
        ]
        self.assertEqual(offset_pos, expected_pos)

        # Negative offset with clamping
        offset_neg = apply_offset(segments, -3.0)
        expected_neg = [
            (0.0, 2.0, 'MANUAL'),  # -3.0 clamped to 0.0, 5.0 - 3.0 = 2.0
            (2.0, 7.0, 'AUTONOMOUS'),
        ]
        self.assertEqual(offset_neg, expected_neg)

        # Negative offset completely filtering out early segments
        offset_large_neg = apply_offset(segments, -7.0)
        expected_large_neg = [
            (0.0, 3.0, 'AUTONOMOUS'),  # Segment 1 is completely before 0 (-7.0 to -2.0) so discarded
        ]
        self.assertEqual(offset_large_neg, expected_large_neg)

    def test_e2e_integration_missing_file(self):
        with self.assertRaises(FileNotFoundError):
            process_log_to_subtitles('non_existent_file.log', 'output.srt')


if __name__ == '__main__':
    unittest.main()
