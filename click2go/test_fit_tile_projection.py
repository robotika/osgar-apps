"""Checks for empirical tile fitting and its independent far-corner diagnostic."""

import copy
import math
import unittest

import numpy as np
from fit_tile_projection import fit_tile_projection, transform


def synthetic_annotations():
    a = 0.3 / math.sqrt(2)
    ground = np.array([[0, 0], [2 * a, 0], [4 * a, 0], [a, a], [a, -a]])
    matrix = np.array([[960.0, -500.0, 960.0], [0.0, 0.0, 1050.0], [1.0, 0.0, 1.0]])
    pixels = transform(ground, matrix)
    return dict(
        frames={
            str(i): dict(source=dict(file='camera.log', recording_start_utc=100.0), source_size=[1920, 1080])
            for i in range(5)
        },
        clicks=[dict(frame_id=str(i), pixel_xy=p.tolist()) for i, p in enumerate(pixels)],
    )


class TileFitTest(unittest.TestCase):
    def test_recovers_square_geometry_including_held_out_corner(self):
        report = fit_tile_projection(synthetic_annotations())
        self.assertLess(report['fit_rms_m'], 1e-6)
        self.assertLess(report['held_out_check']['error_m'], 1e-6)
        self.assertAlmostEqual(report['points'][1]['expected_floor_xy_m'][0], 0.3 * math.sqrt(2))
        self.assertAlmostEqual(report['points'][3]['expected_floor_xy_m'][1], 0.3 / math.sqrt(2))
        self.assertLess(report['points'][4]['expected_floor_xy_m'][1], 0)

    def test_far_corner_is_independent_of_near_tile_fit(self):
        annotations = synthetic_annotations()
        annotations['clicks'][2]['pixel_xy'][1] -= 30
        original = copy.deepcopy(annotations)
        report = fit_tile_projection(annotations)
        self.assertGreater(report['held_out_check']['error_m'], 0.05)
        self.assertEqual(annotations, original)

    def test_reject_mixed_recordings_and_bad_pixels(self):
        for field, value in (('recording', 'other.log'), ('resolution', [640, 360]), ('pixel', [float('nan'), 800])):
            annotations = synthetic_annotations()
            if field == 'recording':
                annotations['frames']['1']['source']['file'] = value
            elif field == 'resolution':
                annotations['frames']['1']['source_size'] = value
            else:
                annotations['clicks'][1]['pixel_xy'] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                fit_tile_projection(annotations)

    def test_reject_degenerate_geometry_and_invalid_selection(self):
        annotations = synthetic_annotations()
        for i, c in enumerate(annotations['clicks']):
            c['pixel_xy'] = [100 + i * 10, 800]
        with self.assertRaises(ValueError):
            fit_tile_projection(annotations)
        with self.assertRaises(ValueError):
            fit_tile_projection(synthetic_annotations(), tile_size=0)
        with self.assertRaises(ValueError):
            fit_tile_projection(synthetic_annotations(), click_numbers=[0, 1, 2, 3, 4])
