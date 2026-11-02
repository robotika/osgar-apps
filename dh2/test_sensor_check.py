import datetime
import tempfile
import unittest
from pathlib import Path

import numpy as np
from osgar.lib.quaternion import euler_to_quaternion
from osgar.lib.serialize import serialize
from osgar.logger import LogWriter

from dh2.sensor_check import IMU_WIDTHS, orientation_degrees, read_log, sample_statistics


class SensorCheckTest(unittest.TestCase):
    def write_log(self, path, *, imu=True, invalid=False):
        names = [*IMU_WIDTHS, 'oak.depth_seq', 'oak.depth', 'oak.color',
                 'platform.pose2d', 'platform.rpy', 'platform.joint_angle']
        with LogWriter(filename=str(path)) as log:
            ids = {n: log.register(n) for n in names}
            for name, value in [('oak.depth_seq', [1, 123000000]),
                                ('oak.depth', np.ones((4, 6), dtype=np.uint16) * 1000),
                                ('oak.color', b'not decoded'), ('platform.pose2d', [0, 0, 0]),
                                ('platform.rpy', [0, 0, 9000]), ('platform.joint_angle', [0])]:
                log.write(ids[name], serialize(value), dt=datetime.timedelta(seconds=1))
            if not imu:
                return
            for batch in range(2):
                values = {n: [] for n in IMU_WIDTHS}
                for i in range(batch * 5, (batch + 1) * 5):
                    sec = 100 + i / 100
                    q = euler_to_quaternion(i / 100, 0, 0)
                    if i >= 5:
                        q = [-x for x in q]
                    values['oak.orientation_list'].append([sec, 0, *q])
                    values['oak.acc_raw_list'].append([sec, 0, 0, 9.81])
                    values['oak.gyro_raw_list'].append([sec, 0, 0, 1])
                if invalid:
                    values['oak.orientation_list'][0] = [100, 0, 0, 0, 0, 0]
                    values['oak.acc_raw_list'].append([100, float('nan'), 0, 0])
                    values['oak.gyro_raw_list'].append([1, 2])
                for name, rows in values.items():
                    log.write(ids[name], serialize(rows), dt=datetime.timedelta(seconds=1.05 + batch * .05))

    def test_batched_device_timing_and_quaternion_sign(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.log'
            self.write_log(path)
            report, data = read_log(path)
        self.assertTrue(report['capture_ready'])
        for stats in report['imu'].values():
            self.assertEqual(stats['samples'], 10)
            self.assertEqual(stats['batches'], 2)
            self.assertAlmostEqual(stats['mean_hz'], 100)
            self.assertAlmostEqual(stats['max_gap_seconds'], .01)
        angles = orientation_degrees(data['oak.orientation_list'])
        np.testing.assert_allclose(angles[:, 2], np.degrees(np.arange(10) / 100), atol=1e-10)
        self.assertEqual(report['frames'], {'oak.color': 1, 'oak.depth': 1})

    def test_historical_log_without_imu_is_reported_not_crashed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'old.log'
            self.write_log(path, imu=False)
            report, _ = read_log(path)
        self.assertFalse(report['capture_ready'])
        self.assertIn('oak.orientation_list: no samples', report['issues'])
        self.assertEqual(report['messages']['platform.pose2d'], 1)

    def test_malformed_samples_and_zero_quaternion_fail_capture_check(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bad.log'
            self.write_log(path, invalid=True)
            report, data = read_log(path)
        self.assertFalse(report['capture_ready'])
        self.assertEqual(report['imu']['oak.acc_raw_list']['malformed'], 2)
        self.assertEqual(report['imu']['oak.gyro_raw_list']['malformed'], 2)
        self.assertTrue(any('quaternion norm' in issue for issue in report['issues']))
        angles = orientation_degrees(data['oak.orientation_list'])
        self.assertTrue(np.isnan(angles[0]).all())
        self.assertTrue(np.isfinite(angles[1:5]).all())

    def test_timestamp_resets_and_large_gaps(self):
        stats = sample_statistics([[1, 10, 0], [2, 10.01, 0], [3, 9, 0], [4, 9, 0], [5, 11, 0]])
        self.assertEqual(stats['nonincreasing_timestamps'], 2)
        self.assertEqual(stats['gaps_over_100ms'], 1)
        self.assertEqual(stats['max_gap_seconds'], 2)
        self.assertEqual(sample_statistics([])['samples'], 0)


if __name__ == '__main__':
    unittest.main()
