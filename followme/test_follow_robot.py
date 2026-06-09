import unittest
from unittest.mock import MagicMock, call
import math
from datetime import timedelta
import numpy as np

from follow_robot import FollowRobot
from osgar.followme import EmergencyStopException


class TestFollowRobot(unittest.TestCase):
    def setUp(self):
        self.bus = MagicMock()
        self.config = {
            'max_speed': 0.5,
            'target_distance': 1.0,
            'Kp_distance': 0.5,
            'Kp_steering': 2.0,
            'horizon': 200,
            'max_track_dist': 3.0,
            'depth_height': 60,
            'terminate_on_stop': True,
            'env': {'OSGAR_LOGS_PREFIX': 'm01-'},
            'verbose': False
        }
        self.node = FollowRobot(self.config, self.bus)
        self.node.time = timedelta(seconds=0)

    def test_initialization(self):
        self.assertEqual(self.node.max_speed, 0.5)
        self.assertEqual(self.node.target_distance, 1.0)
        self.assertEqual(self.node.Kp_distance, 0.5)
        self.assertEqual(self.node.Kp_steering, 2.0)
        self.assertEqual(self.node.horizon, 200)
        self.assertFalse(self.node.status_ready)

    def test_on_emergency_stop(self):
        # When emergency stop is triggered, it should publish green LEDs, stop speed, and raise exception if configured
        with self.assertRaises(EmergencyStopException):
            self.node.on_emergency_stop(True)

        self.node.raise_exception_on_stop = False
        self.node.on_emergency_stop(True)
        # Should publish leds and zero steering
        self.bus.publish.assert_any_call('set_leds', [1, 0, 0, 0])
        self.bus.publish.assert_any_call('set_leds', [0, 0, 128, 0])
        self.bus.publish.assert_any_call('desired_steering', [0, 0])

    def test_on_bumpers(self):
        self.node.on_bumpers_front(True)
        self.bus.publish.assert_any_call('desired_steering', [0, 0])

        self.node.on_bumpers_rear(True)
        self.bus.publish.assert_any_call('desired_steering', [0, 0])

    def test_get_steering_angle(self):
        # If target is in center, steering should be 0
        self.assertAlmostEqual(self.node.get_steering_angle(320, 320), 0.0)

        # If target is to the left (target_x < center_x), steering should be positive
        angle_left = self.node.get_steering_angle(160, 320)
        self.assertTrue(angle_left > 0.0)

        # If target is to the right (target_x > center_x), steering should be negative
        angle_right = self.node.get_steering_angle(480, 320)
        self.assertTrue(angle_right < 0.0)

        # Mirror symmetry
        self.assertAlmostEqual(angle_left, -angle_right)

    def test_on_depth_no_target(self):
        # A depth map where all values are far or invalid (0 or 5000)
        depth_data = np.full((400, 640), 5000, dtype=np.uint16)
        depth_data[100:300, 100:300] = 0 # some shadowed pixels

        self.node.on_depth(depth_data)

        # No target should be successfully updated since min_d > 3.0
        self.assertIsNone(self.node.last_target_x)
        self.assertIsNone(self.node.last_distance)
        self.assertTrue(self.node.status_ready)

        # Check leds set on first depth
        self.bus.publish.assert_any_call('set_leds', [1, 0, 0, 127])
        self.bus.publish.assert_any_call('set_leds', [0, 0, 0, 127])

        # Check subsampled scan published (length 32)
        scan_call = [call for call in self.bus.publish.call_args_list if call[0][0] == 'scan'][0]
        scan_data = scan_call[0][1]
        self.assertEqual(len(scan_data), 32)
        self.assertTrue(all(val in (5000, 10000) for val in scan_data))

    def test_on_depth_with_target_centered(self):
        # Depth map with a target at 1.5 meters, centered at column 320
        depth_data = np.full((400, 640), 5000, dtype=np.uint16)
        # Create a 40px wide target at 1500mm
        depth_data[170:230, 300:340] = 1500

        self.node.on_depth(depth_data)

        # Target should be detected near center
        self.assertIsNotNone(self.node.last_target_x)
        self.assertAlmostEqual(self.node.last_target_x, 320.0, delta=2.0)
        self.assertAlmostEqual(self.node.last_distance, 1.5, delta=0.1)
        self.assertEqual(self.node.last_target_time, self.node.time)

    def test_dynamic_horizon_pitch_adjustment(self):
        # Setting pitch from IMU
        self.node.on_rotation([0, 1000, 0]) # Pitch of +10 degrees
        self.assertEqual(self.node.pitch, 1000)

        depth_data = np.full((400, 640), 5000, dtype=np.uint16)
        # Place target at the shifted horizon (horizon is 200 - 10 * 9.09 = 110)
        # So slice will look from 80 to 140. Place obstacle at Y=100
        depth_data[90:110, 300:340] = 1500

        self.node.on_depth(depth_data)

        # Target should be successfully tracked because the dynamic horizon adjusted to include Y=100
        self.assertIsNotNone(self.node.last_target_x)
        self.assertAlmostEqual(self.node.last_target_x, 320.0, delta=2.0)

    def test_control_loop_on_pose2d(self):
        # 1. Target actively tracked (age <= 0.1s)
        self.node.last_target_x = 300.0
        self.node.last_distance = 1.6 # 1.6m (error of +0.6m from target 1.0m)
        self.node.last_target_time = self.node.time

        self.node.on_pose2d([0, 0, 0])
        # Speed = Kp_distance * (1.6 - 1.0) = 0.5 * 0.6 = 0.3 m/s
        # Steering: target_x < center_x, so steering should be positive
        self.bus.publish.assert_any_call('desired_steering', [300, 431]) # 0.3 * 1000 = 300 mm/s, steering rounded

        # 2. Lock retention (age > 0.1s and <= 1.0s)
        # Advance time by 0.5s
        self.node.time = timedelta(seconds=0.5)
        self.bus.reset_mock()
        self.node.on_pose2d([0, 0, 0])

        # Speed must decelerate to 0 m/s for safety
        # Steering is held but decayed: decay = 1 - 0.5 = 0.5
        self.bus.publish.assert_any_call('desired_steering', [0, 216]) # speed = 0, steering decayed

        # 3. Target completely lost (age > 1.0s)
        # Advance time to 1.5s
        self.node.time = timedelta(seconds=1.5)
        self.bus.reset_mock()
        self.node.on_pose2d([0, 0, 0])

        # Speed and steering should be 0, tracking state reset
        self.bus.publish.assert_any_call('desired_steering', [0, 0])
        self.assertIsNone(self.node.last_target_x)
        self.assertIsNone(self.node.last_distance)

    def test_on_depth_clustering_with_target(self):
        # Configure node with algorithm='clustering'
        clustering_config = self.config.copy()
        clustering_config['algorithm'] = 'clustering'
        node = FollowRobot(clustering_config, self.bus)
        node.time = timedelta(seconds=0)

        # 1. First frame: target detected, count goes to 1, no active tracking yet
        depth_data = np.full((400, 640), 5000, dtype=np.uint16)
        # Create a cluster: height 130 (150 to 280), width 160 (240 to 400)
        depth_data[150:280, 240:400] = 1500  # 1.5m

        node.on_depth(depth_data)
        self.assertEqual(node.tracker_candidate_count, 1)
        self.assertIsNone(node.last_target_x)

        # 2. Second frame: target overlaps, count goes to 2, still no active tracking
        node.time += timedelta(seconds=0.1)
        node.on_depth(depth_data)
        self.assertEqual(node.tracker_candidate_count, 2)
        self.assertIsNone(node.last_target_x)

        # 3. Third frame: target overlaps, count goes to 3, promoted to active tracking!
        node.time += timedelta(seconds=0.1)
        node.on_depth(depth_data)
        self.assertEqual(node.tracker_candidate_count, 3)
        self.assertIsNotNone(node.last_target_x)
        self.assertAlmostEqual(node.last_target_x, 320.0, delta=2.0)
        self.assertAlmostEqual(node.last_distance, 1.5, delta=0.1)

    def test_clustering_led_states(self):
        clustering_config = self.config.copy()
        clustering_config['algorithm'] = 'clustering'
        node = FollowRobot(clustering_config, self.bus)
        node.time = timedelta(seconds=0)

        # 1. Initially (searching), pose2d should publish orange (blinking)
        self.bus.reset_mock()
        node.on_pose2d([0, 0, 0])
        self.bus.publish.assert_any_call('set_leds', [1, 255, 127, 0])
        self.bus.publish.assert_any_call('set_leds', [0, 255, 127, 0])

        # 2. Actively tracked (age <= 0.1s) -> Solid Blue [0, 0, 255]
        self.bus.reset_mock()
        node.last_target_x = 320.0
        node.last_distance = 1.5
        node.last_target_time = node.time
        node.on_pose2d([0, 0, 0])
        self.bus.publish.assert_any_call('set_leds', [1, 0, 0, 255])
        self.bus.publish.assert_any_call('set_leds', [0, 0, 0, 255])

        # 3. Lock retention (age = 0.5s) -> Blinking Orange/Yellow
        node.time += timedelta(seconds=0.5)
        self.bus.reset_mock()
        node.on_pose2d([0, 0, 0])
        self.bus.publish.assert_any_call('set_leds', [1, 255, 127, 0])
        self.bus.publish.assert_any_call('set_leds', [0, 255, 127, 0])

        # 4. Completely lost (age > 1.0s) -> Solid Red [255, 0, 0]
        node.time += timedelta(seconds=1.5)
        self.bus.reset_mock()
        node.on_pose2d([0, 0, 0])
        self.bus.publish.assert_any_call('set_leds', [1, 255, 0, 0])
        self.bus.publish.assert_any_call('set_leds', [0, 255, 0, 0])


if __name__ == '__main__':
    unittest.main()
