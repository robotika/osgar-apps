import json
import unittest
from unittest.mock import MagicMock

from dh2.capture_calibration import capture


class CaptureCalibrationTest(unittest.TestCase):
    def test_factory_geometry_and_explicit_metric_imu_translation(self):
        device, dai = MagicMock(), MagicMock()
        device.getDeviceId.return_value = 'test-camera'
        dai.__version__ = 'test-version'
        calibration = device.readCalibration.return_value

        def export(path):
            path.write_text(json.dumps({'boardName': 'test-board'}))
            return True

        calibration.eepromToJsonFile.side_effect = export
        calibration.getCameraIntrinsics.return_value = [[400, 0, 320], [0, 400, 200], [0, 0, 1]]
        calibration.getStereoLeftRectificationRotation.return_value = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        calibration.getImuToCameraExtrinsics.side_effect = RuntimeError('no IMU calibration')
        result = capture(device, dai)
        self.assertEqual(result['device_id'], 'test-camera')
        self.assertEqual(result['eeprom']['boardName'], 'test-board')
        self.assertIn('left_rectification_rotation', result['queried'])
        self.assertEqual(result['unavailable'], {'imu_to_left_metres': 'no IMU calibration'})
        calibration.getImuToCameraExtrinsics.assert_called_once_with(
            dai.CameraBoardSocket.CAM_B, unit=dai.LengthUnit.METER)

    def test_failed_eeprom_export_is_not_reported_as_success(self):
        device = MagicMock()
        device.readCalibration.return_value.eepromToJsonFile.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'EEPROM'):
            capture(device, MagicMock())


if __name__ == '__main__':
    unittest.main()
