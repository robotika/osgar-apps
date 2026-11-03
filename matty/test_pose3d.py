import math
import unittest
from datetime import timedelta
from unittest.mock import MagicMock

import numpy as np
from osgar.lib import quaternion as quat

from matty.pose3d import Attitudes, MattyPose3D, PoseEstimator, angle_difference, planar_step


def config():
    return dict(wheel_order=[0, 1, 2, 3], front_axle=.16, rear_axle=.16, track=.315,
                housing_offset=[.21, 0, .16], front_from_fused=quat.identity(), rear_from_esp=quat.identity())


def rotation(angle):
    return np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])


def synthetic_distances(displacement, front_old, front, rear_old, rear):
    result = []
    for old, new, axle in [(front_old, front, .16), (rear_old, rear, -.16)]:
        mid = old + angle_difference(new, old) / 2
        forward = np.array([math.cos(mid), math.sin(mid)])
        for side in [.315 / 2, -.315 / 2]:
            lever = (rotation(new) - rotation(old)) @ [axle, side]
            result.append(float(forward @ (displacement + lever)))
    return result


class GeometryTest(unittest.TestCase):
    def test_forward_reverse_and_zero_heading(self):
        for distance in [.1, -.1, 0]:
            step, rms = planar_step([distance] * 4, 0, 0, 0, 0, .16, .16, .315)
            np.testing.assert_allclose(step, [distance, 0], atol=1e-12)
            self.assertLess(rms, 1e-12)

    def test_zero_propulsion_articulation_moves_joint_and_housing(self):
        # A symmetric articulation has zero mean rolling travel but a lateral
        # joint displacement to satisfy both axle nonholonomic constraints.
        front, rear = math.radians(15), -math.radians(15)
        lateral = -.32 * math.sin(front / 2) / math.cos(front / 2)
        expected = np.array([0, lateral])
        distances = synthetic_distances(expected, 0, front, 0, rear)
        self.assertAlmostEqual(sum(distances), 0)
        step, rms = planar_step(distances, 0, front, 0, rear, .16, .16, .315)
        np.testing.assert_allclose(step, expected, atol=1e-12)
        self.assertLess(rms, 1e-12)
        housing = step + rotation(front) @ [.21, 0]
        self.assertGreater(abs(housing[1]), .005)
        self.assertLess(abs(housing[1]), abs(.21 * math.sin(front)))

    def test_small_constant_joint_turn_and_wrap(self):
        # Constant-articulation circle: select the joint chord implied by both
        # axle constraints and independently generate wheel travel from it.
        for initial in [0, math.radians(179)]:
            f0, r0 = initial + .15, initial - .15
            f1, r1 = f0 + .04, r0 + .04
            matrix, values = [], []
            for old, new, axle in [(f0, f1, .16), (r0, r1, -.16)]:
                mid = old + .02
                lateral = np.array([-math.sin(mid), math.cos(mid)])
                matrix.append(lateral)
                values.append(-lateral @ ((rotation(new) - rotation(old)) @ [axle, 0]))
            expected = np.linalg.solve(matrix, values)
            distances = synthetic_distances(expected, f0, f1, r0, r1)
            step, rms = planar_step(distances, f0, (f1 + math.pi) % (2 * math.pi) - math.pi,
                                     r0, r1, .16, .16, .315)
            np.testing.assert_allclose(step, expected, atol=1e-12)
            self.assertLess(rms, 1e-12)

    def test_wheel_disagreement_is_reported_not_called_slip(self):
        _, rms = planar_step([.1, .1, .1, .2], 0, 0, 0, 0, .16, .16, .315)
        self.assertGreater(rms, .02)


class EstimatorTest(unittest.TestCase):
    def test_encoder_wrap_fault_rebase_and_continuity(self):
        model = PoseEstimator(config())
        q = quat.identity()
        model.update(1, [128, 0, 0, [65530] * 4], q, q)
        first = model.update(1.1, [128, 100, 0, [4] * 4], q, q)
        np.testing.assert_allclose(first['pose3d'][0], [.01, 0, 0])
        model.update(1.2, [0x90, 999, 0, [4000] * 4], q, q)
        recovery = model.update(1.3, [128, 100, 0, [4010] * 4], q, q)
        self.assertIn('after_encoder_error', recovery['pose_quality']['reasons'])
        final = model.update(1.4, [128, 100, 0, [4020] * 4], q, q)
        np.testing.assert_allclose(final['pose3d'][0], [.02, 0, 0])
        self.assertFalse(final['pose_quality']['translation_continuous'])

    def test_separate_front_rear_roll_and_housing_lever_arm(self):
        model = PoseEstimator(config())
        front = quat.euler_to_quaternion(0, 0, .3)
        rear = quat.euler_to_quaternion(0, 0, -.2)
        output = model.update(1, [128, 0, 0, [0] * 4], front, rear)
        self.assertAlmostEqual(quat.euler_zyx(output['pose3d'][1])[2], .3)
        self.assertAlmostEqual(quat.euler_zyx(output['rear_pose3d'][1])[2], -.2)
        np.testing.assert_allclose(output['housing_pose3d'][0], [.21, -.16 * math.sin(.3), .16 * math.cos(.3)])
        self.assertFalse(output['pose_quality']['z_observed'])

    def test_initial_alignment_is_fixed_and_joint_not_added_twice(self):
        model = PoseEstimator(config())
        f = quat.euler_to_quaternion(1, 0, 0)
        r = quat.euler_to_quaternion(-2, 0, 0)
        out = model.update(1, [128, 0, 3000, [0] * 4], f, r)
        self.assertAlmostEqual(quat.heading(out['pose3d'][1]), math.radians(15))
        out = model.update(1.1, [128, 0, 3000, [0] * 4], f, r)
        self.assertAlmostEqual(quat.heading(out['pose3d'][1]), math.radians(15))


class SchedulingTest(unittest.TestCase):
    def test_shortest_quaternion_path_gap_and_clock_reset(self):
        buffer = Attitudes(.3)
        a = quat.euler_to_quaternion(math.radians(179), 0, 0)
        b = [-v for v in quat.euler_to_quaternion(math.radians(-179), 0, 0)]
        buffer.add(1, a)
        buffer.add(1.2, b)
        self.assertAlmostEqual(abs(quat.heading(buffer.at(1.1))), math.pi)
        self.assertIsNone(buffer.at(.9))
        self.assertIsNone(buffer.at(1.3))
        with self.assertRaises(ValueError):
            buffer.add(1.1, a)
        buffer.add(2, a)
        self.assertIsNone(buffer.at(1.5))

    def test_delayed_batch_processes_odometry_at_its_own_time(self):
        bus = MagicMock()
        node = MattyPose3D(config(), bus)
        node.time = timedelta(seconds=1)
        node.on_rotation([0, 0, 0])
        node.on_odometry([128, 0, 0, [0] * 4])
        self.assertFalse(bus.publish.called)
        # A configured offset can align individual acquisition times to host.
        node.imu_offset = 0
        node.time = timedelta(seconds=1.1)
        node.on_orientation_list([[.95, 0, 0, 0, 0, 1], [1.05, 0, 0, 0, 0, 1]])
        quality = bus.publish.call_args_list[-1].args[1]
        self.assertEqual(quality['data_time_s'], 1)
        self.assertEqual(quality['publish_time_s'], 1.1)
        self.assertFalse(quality['usable_for_driving'])
        self.assertEqual(set(c.args[0] for c in bus.publish.call_args_list),
                         {'pose3d', 'rear_pose3d', 'housing_pose3d', 'pose_quality'})

    def test_batch_clock_preserves_individual_offsets(self):
        node = MattyPose3D(config(), MagicMock())
        node.time = timedelta(seconds=10)
        node.on_orientation_list([[2, 0, 0, 0, 0, 1], [2.01, 0, 0, 0, 0, 1]])
        np.testing.assert_allclose([t for t, _ in node.attitudes.samples], [9.99, 10])
        self.assertEqual(node.clock_mode, 'first_batch_receipt_approximation')


if __name__ == '__main__':
    unittest.main()
