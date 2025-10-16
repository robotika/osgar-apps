import unittest
from unittest.mock import MagicMock, call

import numpy as np

from dtc import geo_length, latlon2xy


class DTCTest(unittest.TestCase):

    def test_geo_length(self):
        lat1, lon1 = 32.50053483, -83.75835683
        lat2, lon2 = 32.500504, -83.758347
        pos1 = latlon2xy(lat1, lon1)
        pos2 = latlon2xy(lat2, lon2)
        self.assertAlmostEqual(geo_length(pos1, pos2), 3.428055074790579)


if __name__ == '__main__':
    unittest.main()
