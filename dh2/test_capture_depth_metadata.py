import unittest
from datetime import timedelta
from unittest.mock import MagicMock

from dh2.capture_depth_metadata import configure_stereo, frame_metadata, paired_times


class DepthMetadataTest(unittest.TestCase):
    def test_paired_clocks_preserve_both_origins(self):
        packet = MagicMock()
        packet.getTimestamp.return_value = timedelta(seconds=1200.25)
        packet.getTimestampDevice.return_value = timedelta(seconds=3.25)
        self.assertEqual(paired_times(packet), dict(sdk_host_s=1200.25, device_s=3.25,
                                                   sdk_host_minus_device_s=1197))

    def test_absent_frame_transform_does_not_substitute_factory_intrinsics(self):
        frame = MagicMock()
        frame.getTimestamp.return_value = frame.getTimestampDevice.return_value = timedelta(0)
        transform = frame.getTransformation.return_value
        transform.isValid.return_value = False
        result = frame_metadata(frame)
        self.assertIsNone(result['intrinsics'])
        transform.getIntrinsicMatrix.assert_not_called()

    def test_frame_intrinsics_are_validated_and_preserved(self):
        frame = MagicMock()
        frame.getTimestamp.return_value = frame.getTimestampDevice.return_value = timedelta(0)
        frame.getWidth.return_value, frame.getHeight.return_value = 640, 400
        transform = frame.getTransformation.return_value
        matrix = [[410, 0, 315], [0, 409, 195], [0, 0, 1]]
        transform.getIntrinsicMatrix.return_value = matrix
        result = frame_metadata(frame)
        self.assertEqual(result['intrinsics'], matrix)
        transform.getIntrinsicMatrix.return_value = [[0, 0, 0]] * 3
        with self.assertRaises(ValueError):
            frame_metadata(frame)

    def test_configuration_keeps_false_overrides_and_does_not_guess_legacy_preset(self):
        stereo, dai = MagicMock(), MagicMock()
        configure_stereo(stereo, dict(stereo_subpixel=False, stereo_mode='HIGH_ACCURACY'), dai)
        stereo.setSubpixel.assert_called_once_with(False)
        stereo.setDefaultProfilePreset.assert_not_called()
        stereo.setDepthAlign.assert_called_once_with(dai.CameraBoardSocket.CAM_B)
        with self.assertRaises(ValueError):
            configure_stereo(stereo, dict(color_depth_alignment=True), dai)


if __name__ == '__main__':
    unittest.main()
