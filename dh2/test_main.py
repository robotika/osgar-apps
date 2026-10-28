import datetime
import json
import unittest
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np

from dh2.main import Castle


class CastleTest(unittest.TestCase):
    def test_shared_config_selects_side_from_environment(self):
        path = Path(__file__).parent / 'config' / 'dh2.json'
        config = json.loads(path.read_text())['robot']
        self.assertIsNone(config['env']['OSGAR_LOGS_PREFIX'])
        settings = config['modules']['app']['init']
        for prefix, side in [('m04-', 'left'), ('m05-', 'right'), (None, 'left'), ('other-', 'left')]:
            with self.subTest(prefix=prefix):
                node = Castle(dict(settings, env={'OSGAR_LOGS_PREFIX': prefix}), MagicMock())
                self.assertEqual(node.side, side)

    def setUp(self):
        self.bus = MagicMock()
        self.node = Castle({'geometry_verified': True}, self.bus)
        self.node.time = datetime.timedelta(seconds=1)
        self.node.on_bumpers_front(False)
        self.node.on_bumpers_rear(False)
        self.node.on_pose2d([0, 0, 0])
        self.node.on_depth(np.full((400, 640), 3000))

    def command(self):
        return [c.args[1] for c in self.bus.publish.call_args_list
                if c.args[0] == 'desired_steering'][-1]

    def test_stop_cannot_be_overridden(self):
        self.node.on_tick(None)
        self.assertGreater(self.command()[0], 0)
        self.node.on_bumpers_front(True)
        self.node.on_bumpers_front(False)
        self.node.on_depth(np.full((400, 640), 3000))
        self.node.on_tick(None)
        self.assertEqual(self.command(), [0, 0])

    def test_stale_and_invalid_depth(self):
        self.node.time += datetime.timedelta(seconds=1)
        self.node.on_tick(None)
        self.assertEqual(self.command(), [0, 0])
        self.node.on_pose2d([0, 0, 0])
        self.node.on_depth(np.zeros((400, 640)))
        self.node.on_tick(None)
        self.assertEqual(self.command(), [0, 0])

    def test_emergency_stop_latches(self):
        self.node.on_emergency_stop(True)
        self.node.on_emergency_stop(False)
        self.node.on_tick(None)
        self.assertEqual(self.command(), [0, 0])

    def test_parallel_yolo_does_not_erase_cone(self):
        cone = ['cone', 0.9, [0.4, 0.4, 0.6, 0.6]]
        self.node.on_detections([cone])
        self.node.on_detections([])
        self.node.on_detections([['person', 0.9, [0, 0, 1, 1]]])
        self.assertIsNotNone(self.node.target())
        self.node.time += datetime.timedelta(seconds=1)
        self.assertIsNone(self.node.target())

    def test_empty_bbox(self):
        self.node.on_detections([['cone', 0.9, [0.4, 0.4, 0.4, 0.4]]])
        self.assertIsNone(self.node.target())

    def test_confirm_once(self):
        for i in range(35):
            self.node.time = datetime.timedelta(seconds=1 + i / 10)
            self.node.on_pose2d([0, 0, 0])
            self.node.on_depth(np.full((400, 640), 900))
            self.node.on_detections([['cone', 0.9, [0.4, 0.4, 0.6, 0.6]]])
            self.node.on_tick(None)
        reports = [c for c in self.bus.publish.call_args_list if c.args[0] == 'defender']
        self.assertEqual(len(reports), 1)

    def test_side_preference(self):
        for side, sign in [('left', 1), ('right', -1)]:
            self.node.sign = sign
            self.node.clearance_scan = lambda: [2, 0.7, 2]
            self.node.on_tick(None)
            self.assertGreater(self.command()[1] * sign, 0, side)

    def test_geometry_gate(self):
        self.node.geometry_verified = False
        self.node.on_detections([['cone', 0.9, [0.4, 0.4, 0.6, 0.6]]])
        self.assertIsNone(self.node.target())


if __name__ == '__main__':
    unittest.main()
