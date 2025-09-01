import unittest

from report import DTCReport, pack_data, unpack_data


class DTCReportTest(unittest.TestCase):

    def test_usage(self):
        r = DTCReport('m03-',49.911534, 14.199770833333334)
        data = pack_data(r)
        unpacked = unpack_data(data)
        self.assertAlmostEqual(r.location_lat, unpacked.location_lat, 6)
        self.assertAlmostEqual(r.location_lon, unpacked.location_lon, 6)
        self.assertIsNone(unpacked.severe_hemorrhage)

    def test_bits(self):
        r = DTCReport('m03-',49.911534, 14.199770833333334)
        r.severe_hemorrhage = 1
        r.respiratory_distress = 0
        r.hr = 69
        r.rr = 13
        r.trauma_head = 1
        r.trauma_torso = 2

        r.trauma_lower_ext = 3
        r.trauma_upper_ext = 1
        r.alertness_ocular = 0
        r.alertness_verbal = 3
        r.alertness_motor = 2

        data = pack_data(r)
        self.assertEqual(len(data), 15)  # full size
        unpacked = unpack_data(data)

        self.assertIsNotNone(unpacked.severe_hemorrhage)
        self.assertIsNotNone(unpacked.respiratory_distress)
        self.assertEqual(r.hr, unpacked.hr)
        self.assertEqual(r.rr, unpacked.rr)
        self.assertEqual(r.trauma_head, unpacked.trauma_head)
        self.assertEqual(r.trauma_torso, unpacked.trauma_torso)

        self.assertEqual(r.trauma_lower_ext, unpacked.trauma_lower_ext)
        self.assertEqual(r.trauma_upper_ext, unpacked.trauma_upper_ext)
        self.assertEqual(r.alertness_ocular, unpacked.alertness_ocular)
        self.assertEqual(r.alertness_verbal, unpacked.alertness_verbal)
        self.assertEqual(r.alertness_motor, unpacked.alertness_motor)

    def test_out_of_range(self):
        r = DTCReport('m01-',49.911534, 14.199770833333334)
        r.rr = 100
        with self.assertRaises(ValueError):
            pack_data(r)

    def test_system(self):
        r = DTCReport('m03-', 49.911534, 14.199770833333334)
        self.assertEqual(r.system, 'Matty M03')
        packed = pack_data(r)
        self.assertEqual(packed[0], ((ord('M')-ord('A')) << 3) + 3)
        new_r = unpack_data(packed)
        self.assertEqual(new_r.system, 'Matty M03')

    def test_json(self):
        r = DTCReport('m03-',49.911534, 14.199770833333334)
        json_data = r.tojson()
        self.assertAlmostEqual(json_data['location']['latitude'], 49.911534)
        self.assertAlmostEqual(json_data['location']['longitude'], 14.199770833333334)
        self.assertNotIn('severe_hemorrhage', json_data)

        r.severe_hemorrhage = 0
        json_data = r.tojson()
        self.assertIn('severe_hemorrhage', json_data)
        self.assertEqual(json_data['severe_hemorrhage']['value'], 0)

    def test_rr_bug(self):
        r = DTCReport('m03-',49.911534, 14.199770833333334)
        r.rr = 15
        json_data = r.tojson()
        self.assertEqual(json_data['rr']['value'], 15)

    def test_pack_invalid_GPS(self):
        # rather pack data with invalid GPS than crash ...
        r = DTCReport('m03-', None, None)
        data = pack_data(r)
        r2 = unpack_data(data)
        self.assertEqual(r2.location_lat, 0)
        self.assertEqual(r2.location_lon, 0)


if __name__ == '__main__':
    unittest.main()
