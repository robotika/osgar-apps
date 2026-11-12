import unittest
from unittest.mock import MagicMock
from main import RerunRoute


class RerunRouteTest(unittest.TestCase):
    def test_init(self):
        bus = MagicMock()
        config = {
            'logfile': None,
            'pose2d_stream': 'platform.pose2d'
        }
        # It should at least not crash on init with no logfile
        app = RerunRoute(config, bus)
        self.assertEqual(app.path, [])


if __name__ == '__main__':
    unittest.main()
