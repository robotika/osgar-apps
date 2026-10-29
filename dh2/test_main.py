import datetime
import json
import unittest
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
from osgar.exceptions import EmergencyStopException

from dh2.main import Castle, ConeApproach, ConeDetector


class ConeApproachTest(unittest.TestCase):
    def setUp(self):
        self.bus = MagicMock()
        self.node = ConeApproach({'terminate_on_stop': True}, self.bus)
        self.node.time = datetime.timedelta(seconds=1)
        self.node.on_emergency_stop(False)
        self.node.on_pose2d([0, 0, 0])

    def command(self):
        return [c.args[1] for c in self.bus.publish.call_args_list
                if c.args[0] == 'desired_steering'][-1]

    def detect(self, bbox=(0.3, 0.2, 0.5, 0.5), count=3):
        for _ in range(count):
            self.node.time += datetime.timedelta(seconds=0.1)
            self.node.on_detections([['cone', 0.9, list(bbox)]])
            self.node.on_tick(None)

    def test_acquire_and_steer(self):
        self.detect(count=2)
        self.assertEqual(self.command(), [0, 0])
        self.detect(count=1)
        self.assertGreater(self.command()[0], 0)
        self.assertGreater(self.command()[1], 0)

    def test_loss_and_detection_silence_stop(self):
        self.detect()
        self.node.time += datetime.timedelta(seconds=0.6)
        self.node.on_tick(None)
        self.assertEqual(self.command(), [0, 0])
        self.detect()
        self.node.on_detections([])
        self.assertEqual(self.command(), [0, 0])

    def test_large_cone_latches_stop(self):
        self.detect(bbox=(0.2, 0.1, 0.7, 0.9))
        self.assertEqual(self.command(), [0, 0])
        self.detect()
        self.assertEqual(self.command(), [0, 0])

    def test_bumper_and_emergency_stop(self):
        self.detect()
        self.node.on_bumpers_front(True)
        self.node.on_bumpers_front(False)
        self.detect()
        self.assertEqual(self.command(), [0, 0])
        with self.assertRaises(EmergencyStopException):
            self.node.on_emergency_stop(True)
        self.assertEqual(self.command(), [0, 0])

    def test_travel_limit(self):
        self.detect()
        self.node.on_pose2d([2100, 0, 0])
        self.assertEqual(self.command(), [0, 0])
        self.detect()
        self.assertEqual(self.command(), [0, 0])

    def test_low_confidence_and_wrong_label_do_not_drive(self):
        for _ in range(4):
            self.node.on_detections([['person', 0.99, [0.3, 0.2, 0.5, 0.5]],
                                     ['cone', 0.5, [0.3, 0.2, 0.5, 0.5]]])
            self.node.on_tick(None)
        self.assertEqual(self.command(), [0, 0])


class ConeDetectorTest(unittest.TestCase):
    def test_stop_ends_run_without_waiting_for_queued_data(self):
        bus = MagicMock()
        node = ConeDetector({'terminate_on_stop': True}, bus)
        bus.listen.side_effect = [
            (datetime.timedelta(seconds=0.245), 'emergency_stop', True),
            (datetime.timedelta(seconds=0.3), 'detections', [['cone', 0.9, [0, 0, 1, 1]]]),
        ]
        node.run()
        bus.shutdown.assert_called_once()
        bus.publish.assert_not_called()
        self.assertEqual(bus.listen.call_count, 1)

    def test_detections_and_stop_release(self):
        bus = MagicMock()
        node = ConeDetector({}, bus)
        node.on_emergency_stop(False)
        bus.shutdown.assert_not_called()
        cone = ['cone', 0.9, [0.4, 0.4, 0.6, 0.6]]
        node.on_detections([cone, ['person', 0.9, [0, 0, 1, 1]]])
        bus.publish.assert_called_with('cones', [cone])
        node.on_detections([])
        bus.publish.assert_called_with('cones', [])

    def test_pressed_stop_requests_shutdown(self):
        bus = MagicMock()
        node = ConeDetector({'terminate_on_stop': True}, bus)
        with self.assertRaises(EmergencyStopException):
            node.on_emergency_stop(True)

    def test_stop_termination_is_opt_in(self):
        bus = MagicMock()
        node = ConeDetector({}, bus)
        node.on_emergency_stop(True)
        bus.shutdown.assert_not_called()


class CastleTest(unittest.TestCase):
    def test_shared_config_selects_side_from_environment(self):
        path = Path(__file__).parent / 'config' / 'dh2-backup.json'
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

    def test_invalid_depth(self):
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
