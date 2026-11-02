"""Read OAK factory calibration and IMU extrinsics; does not start robot motion."""

import argparse
import json
import tempfile
from pathlib import Path


def capture(device, dai):
    calibration = device.readCalibration()
    with tempfile.TemporaryDirectory() as directory:
        raw_path = Path(directory) / 'eeprom.json'
        if not calibration.eepromToJsonFile(raw_path):
            raise RuntimeError('Could not export camera EEPROM calibration')
        eeprom = json.loads(raw_path.read_text())
    result = dict(device_id=device.getDeviceId(), depthai_version=dai.__version__, eeprom=eeprom,
                  queried={}, unavailable={},
                  notes=['Factory camera/IMU geometry, not camera-to-chassis mounting calibration',
                         'Resized camera intrinsics are not asserted to be rectified depth intrinsics'])
    queries = {
        'left_intrinsics_640x400': lambda: calibration.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, 640, 400),
        'left_rectification_rotation': calibration.getStereoLeftRectificationRotation,
        'imu_to_left_metres': lambda: calibration.getImuToCameraExtrinsics(
            dai.CameraBoardSocket.CAM_B, unit=dai.LengthUnit.METER),
    }
    for name, query in queries.items():
        try:
            result['queried'][name] = query()
        except RuntimeError as error:
            result['unavailable'][name] = str(error)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import depthai as dai

    with dai.Device() as device:
        result = capture(device, dai)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(f'Calibration for {result["device_id"]}: {args.output}')
    for name, reason in result['unavailable'].items():
        print(f'Unavailable {name}: {reason}')


if __name__ == '__main__':
    main()
