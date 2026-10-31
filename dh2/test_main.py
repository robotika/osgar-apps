import datetime
import unittest
from unittest.mock import MagicMock

import numpy as np
from osgar.exceptions import EmergencyStopException

from dh2.main import Castle, ConeApproach, ConeDetector, ConeExplore


class ConeMissionTest(unittest.TestCase):
    close_cone = [['cone', 0.95, [0.4, 0.1, 0.6, 0.8]]]

    def setUp(self):
        self.bus = MagicMock()
        self.node = ConeExplore({'terminate_on_stop': True, 'obstacle_stop_dist': 0.7}, self.bus)
        self.node.time = datetime.timedelta(0)
        self.node.on_emergency_stop(False)
        self.node.on_pose2d([0, 0, 0])

    def event(self, seconds, cones=None, depth_mm=None):
        self.node.time = datetime.timedelta(seconds=seconds)
        if depth_mm is not None:
            self.node.on_depth(np.full((400, 640), depth_mm))
        self.node.on_detections([] if cones is None else cones)
        self.node.on_tick(None)

    def reports(self):
        return [c.args[1] for c in self.bus.publish.call_args_list if c.args[0] == 'defender']

    def test_report_only_after_three_seconds_then_depart(self):
        self.event(0, depth_mm=2000)
        for i in range(6, 38):
            self.event(i / 10, self.close_cone)
        self.assertEqual(self.node.state, 'wait')
        self.assertEqual(self.reports(), [])
        self.event(3.8, self.close_cone)
        self.assertEqual(len(self.reports()), 1)
        self.assertEqual(self.reports()[0]['wait_sec'], 3)
        self.assertEqual(self.node.state, 'recover_scan')
        self.assertFalse(self.node.done)
        self.event(3.9, self.close_cone)
        self.assertEqual(len(self.reports()), 1)

    def test_lost_cone_cancels_wait(self):
        self.event(0, depth_mm=2000)
        for t in (0.6, 0.7, 0.8):
            self.event(t, self.close_cone)
        self.event(1.0)
        self.assertEqual(self.node.state, 'scan')
        self.assertEqual(self.reports(), [])

    def test_reported_position_is_not_selected_again(self):
        target = (0.7, 0.5)
        self.node.visited.append(self.node.target_position(target))
        self.assertIsNone(self.node.select_target([target]))
        self.assertEqual(self.node.select_target([(0.1, 0.5)]), (0.1, 0.5))

    def test_four_distinct_visits_stop_search(self):
        for index in range(4):
            base = index * 10
            self.node.pose = (index * 4, 0)
            self.node.travelled = index * 4
            self.node.time = datetime.timedelta(seconds=base)
            self.node.start_scan()
            for i in range(6, 39):
                self.event(base + i / 10, self.close_cone, depth_mm=2000)
        self.assertEqual([r['id'] for r in self.reports()], [1, 2, 3, 4])
        self.assertTrue(self.node.done)

    def test_obstacle_recovery_selects_free_side_and_resumes(self):
        self.event(0, depth_mm=2000)
        self.node.state = 'advance'
        self.node.command(0.2, 0, 'test advance')
        self.event(1, depth_mm=500)
        self.assertEqual(self.node.state, 'recover_scan')
        self.event(2.5, depth_mm=2000)  # left clear
        self.event(4.0, depth_mm=500)  # right blocked
        self.event(5.5, depth_mm=500)  # center blocked
        self.assertEqual(self.node.state, 'recover_orient')
        self.assertGreater(self.node.escape_angle, 0)
        self.event(6.1)  # old depth cannot authorize the new direction
        self.assertEqual(self.node.state, 'recover_orient')
        self.event(6.2, depth_mm=2000)
        self.event(6.3, depth_mm=2000)
        self.assertTrue(self.node.moving)
        self.node.on_pose2d([500, 0, 0])
        self.event(6.4, depth_mm=2000)
        self.assertEqual(self.node.state, 'scan')
        self.assertFalse(self.node.done)

    def test_all_sides_blocked_keep_scanning_without_reverse(self):
        self.event(0, depth_mm=500)
        self.node.start_recovery('obstacle')
        for t in (1.5, 3, 4.5, 6, 7.5, 9):
            self.event(t, depth_mm=500)
        self.assertEqual(self.node.state, 'recover_scan')
        self.assertFalse(self.node.done)
        commands = [c.args[1] for c in self.bus.publish.call_args_list if c.args[0] == 'desired_steering']
        self.assertTrue(all(command[0] == 0 for command in commands))

    def test_front_bumper_stops_without_ending_search(self):
        self.event(0, depth_mm=2000)
        self.node.command(0.2, 0, 'test advance')
        self.node.on_bumpers_front(True)
        self.assertFalse(self.node.moving)
        self.assertFalse(self.node.done)
        self.assertEqual(self.node.state, 'recover_scan')
        self.node.command(0.2, 0, 'must not override contact')
        self.assertFalse(self.node.moving)

    def test_search_has_no_default_travel_limit(self):
        self.event(0, depth_mm=2000)
        self.node.command(0.2, 0, 'test advance')
        self.node.on_pose2d([20000, 0, 0])
        self.assertFalse(self.node.done)

    def begin_reverse(self):
        self.node.allow_reverse = True
        self.event(0, depth_mm=500)
        self.node.start_recovery('obstacle')
        for t in (1.5, 3, 4.5):
            self.event(t, depth_mm=500)
        self.assertEqual(self.node.state, 'reverse')

    def last_command(self):
        return [c.args[1] for c in self.bus.publish.call_args_list
                if c.args[0] == 'desired_steering'][-1]

    def test_rear_contact_stops_reverse_immediately_and_cannot_be_overridden(self):
        self.begin_reverse()
        self.assertEqual(self.last_command(), [-100, 0])
        self.node.on_bumpers_rear(True)
        self.assertEqual(self.last_command()[0], 0)
        self.node.command(-0.1, 0, 'must not override rear contact')
        self.assertEqual(self.last_command()[0], 0)
        self.assertFalse(self.node.done)

    def test_reverse_ends_at_30cm_and_is_not_repeated_without_forward_progress(self):
        self.begin_reverse()
        self.node.on_pose2d([-300, 0, 0])
        self.assertEqual(self.node.state, 'recover_scan')
        self.assertEqual(self.last_command()[0], 0)
        for t in (6, 7.5, 9):
            self.event(t, depth_mm=500)
        self.assertEqual(self.node.state, 'recover_scan')
        self.assertEqual(self.last_command()[0], 0)

    def test_reverse_timeout_bounds_stalled_maneuver(self):
        self.begin_reverse()
        self.event(8.6, depth_mm=500)
        self.assertEqual(self.node.state, 'recover_scan')
        self.assertEqual(self.last_command()[0], 0)

    def test_rear_contact_prevents_starting_reverse(self):
        self.node.allow_reverse = True
        self.event(0, depth_mm=500)
        self.node.on_bumpers_rear(True)
        for t in (1.5, 3, 4.5):
            self.event(t, depth_mm=500)
        self.assertEqual(self.node.state, 'recover_scan')
        self.assertFalse(self.node.moving)

    def test_blocking_cone_can_be_reported_during_recovery_scan(self):
        self.event(0, depth_mm=500)
        self.node.start_recovery('obstacle')
        for i in range(6, 39):
            self.event(i / 10, self.close_cone, depth_mm=500)
        self.assertEqual(len(self.reports()), 1)
        self.assertEqual(self.node.state, 'recover_scan')

    def test_stalled_forward_recovery_rescans(self):
        self.event(0, depth_mm=2000)
        self.node.state = 'escape'
        self.node.recovery_cause = 'obstacle'
        self.node.escape_started = self.node.time
        self.event(6.1, depth_mm=2000)
        self.assertEqual(self.node.state, 'recover_scan')


class ConeExploreTest(unittest.TestCase):
    def setUp(self):
        self.bus = MagicMock()
        self.node = ConeExplore({'terminate_on_stop': True, 'max_travel': 10}, self.bus)
        self.node.time = datetime.timedelta(seconds=0)
        self.node.on_emergency_stop(False)
        self.node.on_pose2d([0, 0, 0])

    def tick(self, seconds, detections=None):
        self.node.time = datetime.timedelta(seconds=seconds)
        self.node.on_detections([] if detections is None else detections)
        self.node.on_tick(None)
        return [c.args[1] for c in self.bus.publish.call_args_list
                if c.args[0] == 'desired_steering'][-1]

    def test_scan_then_advance_then_scan(self):
        self.assertEqual(self.tick(0), [0, 0])
        self.assertEqual(self.tick(1.5), [0, 3000])
        self.assertEqual(self.tick(3), [0, -3000])
        self.assertEqual(self.tick(4.5), [0, 0])
        self.tick(6)
        self.assertEqual(self.tick(6.1), [200, 0])
        self.node.on_pose2d([500, 0, 0])
        self.assertEqual(self.tick(6.2), [0, 0])
        self.assertEqual(self.node.state, 'scan')

    def test_side_acquisition_keeps_articulation_and_uses_faster_speed(self):
        cone = [['cone', 0.9, [0.4, 0.2, 0.6, 0.5]]]
        self.tick(0)
        self.tick(1.5)
        for t in (2.1, 2.2, 2.3):
            command = self.tick(t, cone)
        self.assertEqual(command[1], 3000)
        self.assertGreaterEqual(command[0], 150)
        self.assertLessEqual(command[0], 300)
        self.assertEqual(self.node.state, 'approach')
        self.assertEqual(self.tick(2.4), [0, 0])
        self.assertEqual(self.node.state, 'scan')

    def test_settling_frames_do_not_start_approach(self):
        cone = [['cone', 0.9, [0.4, 0.2, 0.6, 0.5]]]
        self.tick(0)
        for t in (0.1, 0.2, 0.3):
            self.assertEqual(self.tick(t, cone)[0], 0)

    def test_stop_cancels_stationary_steering(self):
        self.tick(0)
        self.tick(1.5)
        with self.assertRaises(EmergencyStopException):
            self.node.on_emergency_stop(True)
        self.assertEqual(self.bus.publish.call_args_list[-2].args,
                         ('desired_steering', [0, 0]))

    def test_speed_cannot_exceed_platform_limit(self):
        node = ConeExplore({'max_speed': 1, 'min_speed': 1, 'explore_speed': 1}, MagicMock())
        self.assertEqual((node.max_speed, node.min_speed, node.explore_speed), (0.5, 0.5, 0.5))

    def enable_obstacle_stop(self):
        self.node = ConeExplore({'terminate_on_stop': True, 'obstacle_stop_dist': 0.7,
                                 'obstacle_resume_dist': 1.0}, self.bus)
        self.node.time = datetime.timedelta(seconds=0)
        self.node.on_emergency_stop(False)
        self.node.on_pose2d([0, 0, 0])

    def test_depth_stop_overrides_cone_approach_and_resumes_with_hysteresis(self):
        self.enable_obstacle_stop()
        cone = [['cone', 0.9, [0.4, 0.2, 0.6, 0.5]]]
        self.node.on_depth(np.full((400, 640), 2000))
        self.tick(0)
        for t in (0.6, 0.7, 0.8):
            command = self.tick(t, cone)
        self.assertGreater(command[0], 0)
        # A new depth frame stops the moving robot without waiting for a tick.
        self.node.on_depth(np.full((400, 640), 500))
        commands = [c.args[1] for c in self.bus.publish.call_args_list if c.args[0] == 'desired_steering']
        self.assertEqual(commands[-1][0], 0)
        self.assertFalse(self.node.done)
        self.node.on_depth(np.full((400, 640), 850))
        self.assertEqual(self.tick(0.9, cone)[0], 0)
        self.node.on_depth(np.full((400, 640), 1200))
        self.assertFalse(self.node.obstacle_blocked)
        self.assertEqual(self.node.state, 'recover_scan')
        self.assertEqual(self.tick(1.0, cone)[0], 0)

    def test_wait_for_initial_depth_then_allow_zero_depth(self):
        self.enable_obstacle_stop()
        self.tick(0)
        self.assertEqual(self.tick(1.5), [0, 3000])
        self.tick(3)
        self.tick(4.5)
        self.tick(6)
        self.assertEqual(self.tick(6.1)[0], 0)
        self.node.on_depth(np.zeros((400, 640)))
        self.assertGreater(self.tick(6.2)[0], 0)
        self.node.on_depth(np.full((400, 640), 2000))
        self.assertGreater(self.tick(6.3)[0], 0)
        self.node.on_depth(np.zeros((400, 640)))
        self.assertGreater(self.tick(6.4)[0], 0)

    def test_sparse_far_depth_does_not_block(self):
        self.enable_obstacle_stop()
        depth = np.zeros((400, 640))
        depth[150:160, 310:320] = 5000
        self.node.on_depth(depth)
        self.assertFalse(self.node.obstacle_blocked)
        self.assertEqual(self.node.obstacle_distance, 5.0)

    def test_close_patch_in_otherwise_zero_depth_still_blocks(self):
        self.enable_obstacle_stop()
        depth = np.zeros((400, 640))
        depth[150:155, 310:315] = 500
        self.node.on_depth(depth)
        self.assertTrue(self.node.obstacle_blocked)
        self.assertEqual(self.node.obstacle_distance, 0.5)
        self.node.on_depth(np.zeros((400, 640)))
        self.assertFalse(self.node.obstacle_blocked)

    def test_single_close_pixel_is_not_enough(self):
        self.enable_obstacle_stop()
        depth = np.full((400, 640), 3000)
        depth[150, 320] = 200
        self.node.on_depth(depth)
        self.assertFalse(self.node.obstacle_blocked)

    def test_ground_below_image_center_is_excluded(self):
        self.enable_obstacle_stop()
        depth = np.full((400, 640), 3000)
        depth[200:, :] = 200
        self.node.on_depth(depth)
        self.assertFalse(self.node.obstacle_blocked)


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
    def test_side_override_from_environment(self):
        settings = {'search_side': 'left', 'm04-search_side': 'left', 'm05-search_side': 'right'}
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
