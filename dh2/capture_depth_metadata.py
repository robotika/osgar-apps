"""Capture OAK frame projection and paired clocks with a camera-only pipeline."""

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from dh2.capture_calibration import capture


def clock_anchor(dai):
    before = datetime.now(timezone.utc).timestamp()
    sdk = dai.Clock.now().total_seconds()
    after = datetime.now(timezone.utc).timestamp()
    return dict(sdk_host_s=sdk, utc_before_s=before, utc_after_s=after,
                utc_minus_sdk_interval_s=[before - sdk, after - sdk])


def paired_times(packet):
    host = packet.getTimestamp().total_seconds()
    device = packet.getTimestampDevice().total_seconds()
    return dict(sdk_host_s=host, device_s=device, sdk_host_minus_device_s=host - device)


def frame_metadata(frame):
    transformation = frame.getTransformation()
    result = dict(sequence=frame.getSequenceNum(), width=frame.getWidth(), height=frame.getHeight(),
                  instance_number=frame.getInstanceNum(), **paired_times(frame),
                  transformation_valid=transformation.isValid(), intrinsics=None)
    if result['transformation_valid']:
        matrix = np.asarray(transformation.getIntrinsicMatrix(), dtype=float)
        if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
                or matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or abs(np.linalg.det(matrix)) < 1e-9):
            raise ValueError('SDK frame reports invalid intrinsics')
        result.update(intrinsics=matrix.tolist(), source_intrinsics=transformation.getSourceIntrinsicMatrix(),
                      distortion_coefficients=transformation.getDistortionCoefficients(),
                      image_transform=transformation.getMatrix())
    return result


def configure_stereo(stereo, settings, dai):
    """Mirror the reviewed v3 driver's stereo overrides; no renamed preset guesses."""
    if settings.get('color_depth_alignment', False):
        raise ValueError('This camera-only capture supports left-aligned depth only')
    if settings.get('depth_profile') is not None:
        stereo.setDefaultProfilePreset(getattr(dai.node.StereoDepth.PresetMode, settings['depth_profile']))
    for key, method in [('stereo_extended_disparity', 'setExtendedDisparity'),
                        ('stereo_left_right_check', 'setLeftRightCheck'),
                        ('stereo_subpixel', 'setSubpixel'),
                        ('set_rectify_edge_fill_color', 'setRectifyEdgeFillColor'),
                        ('enable_distortion_correction', 'enableDistortionCorrection')]:
        if settings.get(key) is not None:
            getattr(stereo, method)(settings[key])
    if settings.get('stereo_median_filter') is not None:
        stereo.initialConfig.setMedianFilter(getattr(dai.MedianFilter, settings['stereo_median_filter']))
    if settings.get('set_left_right_check_threshold') is not None:
        stereo.initialConfig.setLeftRightCheckThreshold(settings['set_left_right_check_threshold'])
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)


def capture_depth(device, dai, settings, frame_count, timeout):
    result = dict(factory=capture(device, dai), oak_settings=settings,
                  ignored_stereo_mode=settings.get('stereo_mode'), frames=[], rectified_left=[], imu=[], anchors=[],
                  limitations=['Camera-only pipeline; mission NN/encoder load is not reproduced',
                               'Clock relationships apply only to this camera boot/session',
                               'SDK-reported frame intrinsics still need physical projection validation',
                               'This does not update mission logging or calibrate the camera mounting'])
    histogram = np.zeros(3001, dtype=np.int64)
    with dai.Pipeline(device) as pipeline:
        outputs = []
        for socket in [dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C]:
            camera = pipeline.create(dai.node.Camera).build(socket)
            if settings.get('stereo_manual_exposure') is not None:
                camera.initialControl.setManualExposure(*settings['stereo_manual_exposure'])
            if settings.get('stereo_manual_wb') is not None:
                camera.initialControl.setManualWhiteBalance(settings['stereo_manual_wb'])
            outputs.append(camera.requestOutput(tuple(settings.get('mono_resolution', [640, 400])),
                                                type=dai.ImgFrame.Type.NV12, fps=settings.get('fps', 10)))
        stereo = pipeline.create(dai.node.StereoDepth)
        outputs[0].link(stereo.left)
        outputs[1].link(stereo.right)
        configure_stereo(stereo, settings, dai)
        depths = stereo.depth.createOutputQueue(blocking=False)
        left_images = stereo.rectifiedLeft.createOutputQueue(blocking=False)
        depth_transforms, left_transforms = {}, {}
        imu = pipeline.create(dai.node.IMU)
        sensor = (dai.IMUSensor.GAME_ROTATION_VECTOR if settings.get('disable_magnetometer_fusion', False)
                  else dai.IMUSensor.ROTATION_VECTOR)
        imu.enableIMUSensor(sensor, 100)
        imu.setBatchReportThreshold(5)
        imu.setMaxBatchReports(20)
        orientations = imu.out.createOutputQueue(blocking=False)
        pipeline.start()
        # The DH2 recordings used passive illumination. Keep this test passive too.
        device.setIrLaserDotProjectorIntensity(0)
        device.setIrFloodLightIntensity(0)
        deadline = time.monotonic() + timeout
        while len(result['frames']) < frame_count:
            if not pipeline.isRunning() or time.monotonic() > deadline:
                raise RuntimeError('Camera capture ended or timed out before requested depth frames arrived')
            result['anchors'].append(clock_anchor(dai))
            for frame in depths.tryGetAll():
                if len(result['frames']) < frame_count:
                    result['frames'].append(frame_metadata(frame))
                    depth_transforms[frame.getSequenceNum()] = frame.getTransformation()
                    samples = frame.getCvFrame()[::8, ::8]
                    valid = samples[(samples >= 300) & (samples <= 3000)]
                    histogram += np.bincount(valid.astype(int), minlength=len(histogram))
            for frame in left_images.tryGetAll():
                if len(result['rectified_left']) < frame_count:
                    result['rectified_left'].append(frame_metadata(frame))
                    left_transforms[frame.getSequenceNum()] = frame.getTransformation()
            for batch in orientations.tryGetAll():
                for packet in batch.packets:
                    orientation = packet.rotationVector
                    result['imu'].append(dict(**paired_times(orientation), quaternion_xyzw=[
                        orientation.i, orientation.j, orientation.k, orientation.real]))
            time.sleep(.01)
        result['anchors'].append(clock_anchor(dai))
    if not result['imu']:
        raise RuntimeError('No paired IMU timestamps captured')
    result['depth_left_alignment'] = [dict(sequence=seq, sdk_reports_aligned=depth_transforms[seq].isAlignedTo(
        left_transforms[seq])) for seq in sorted(depth_transforms.keys() & left_transforms.keys())
        if depth_transforms[seq].isValid() and left_transforms[seq].isValid()]
    result['sampled_depth_histogram_mm'] = {str(int(z)): int(histogram[z]) for z in np.flatnonzero(histogram)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path('dh2/config/dh2.json'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=20)
    parser.add_argument('--timeout', type=float, default=15)
    parser.add_argument('--subpixel', action=argparse.BooleanOptionalAction, default=None,
                        help='Override stereo subpixel mode for this capture only')
    args = parser.parse_args()
    if args.frames < 2 or not np.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('Need at least two frames and a positive finite timeout')
    settings = json.loads(args.config.read_text())['robot']['modules']['oak']['init']
    if args.subpixel is not None:
        settings['stereo_subpixel'] = args.subpixel
    import depthai as dai

    with dai.Device() as device:
        result = capture_depth(device, dai, settings, args.frames, args.timeout)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(f'{len(result["frames"])} depth frames, {len(result["imu"])} IMU samples: {args.output}')
    if not all(frame['transformation_valid'] for frame in result['frames']):
        print('Some frames lack SDK transformation metadata; no intrinsics substituted')


if __name__ == '__main__':
    main()
