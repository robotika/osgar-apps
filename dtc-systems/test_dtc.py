import unittest
from unittest.mock import MagicMock, call

import numpy as np

from dtc import geo_length, latlon2xy, geo_angle


class DTCTest(unittest.TestCase):

    def test_geo_length(self):
        lat1, lon1 = 32.50053483, -83.75835683
        lat2, lon2 = 32.500504, -83.758347
        pos1 = latlon2xy(lat1, lon1)
        pos2 = latlon2xy(lat2, lon2)
        self.assertAlmostEqual(geo_length(pos1, pos2), 3.5517766580136345)

    def test_geo_length2(self):
        # m04-dtc-night-251002_020022.log - first 20s
        lat1, lon1 = 32.500546, -83.75836183333334
        lat2, lon2 = 32.500537333333334, -83.75832816666667
        pos1 = latlon2xy(lat1, lon1)
        pos2 = latlon2xy(lat2, lon2)
        self.assertAlmostEqual(geo_length(pos1, pos2), 3.325751054157847)  # should be ~4m

    def test_geo_angle(self):
        # m04-dtc-night-251002_020022.log - first 20s
        lat1, lon1 = 32.500546, -83.75836183333334
        lat2, lon2 = 32.500537333333334, -83.75832816666667
        pos1 = latlon2xy(lat1, lon1)
        pos2 = latlon2xy(lat2, lon2)
        self.assertAlmostEqual(geo_angle(pos1, pos2), -0.3015198293766636)


if __name__ == '__main__':
    unittest.main()
