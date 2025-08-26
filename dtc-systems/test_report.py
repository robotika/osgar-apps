import unittest

from report import DTCReport, pack_data, unpack_data


class DTCReportTest(unittest.TestCase):

    def test_usage(self):
        r = DTCReport(0, 0)
        data = pack_data(r)
        unpacked = unpack_data(data)


if __name__ == '__main__':
    unittest.main()
