import unittest

from report import DTCReport, pack_data, unpack_data


class DTCReportTest(unittest.TestCase):

    def test_usage(self):
        r = DTCReport(49.911534, 14.199770833333334)
        data = pack_data(r)
        unpacked = unpack_data(data)
        self.assertAlmostEqual(r.location_lat, unpacked.location_lat, 6)
        self.assertAlmostEqual(r.location_lon, unpacked.location_lon, 6)
        self.assertIsNone(unpacked.severe_hemorrhage)

    def test_bits(self):
        r = DTCReport(49.911534, 14.199770833333334)
        r.severe_hemorrhage = 1
        r.respiratory_distress = 0
        data = pack_data(r)
        unpacked = unpack_data(data)
        self.assertIsNotNone(unpacked.severe_hemorrhage)
        self.assertIsNotNone(unpacked.respiratory_distress)


if __name__ == '__main__':
    unittest.main()
