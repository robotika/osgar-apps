#!/usr/bin/python
"""
  View road NN detection mask with the original color image
"""
import datetime
import pathlib
import math
from datetime import timedelta

import cv2
import numpy as np

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize

from main import mask_center
from log_info import get_time_and_dist


def read_h264_image(data, i_frame_only=True):
    assert data.startswith(bytes.fromhex('00000001 0950')) or data.startswith(bytes.fromhex('00000001 0930')), data[
                                                                                                               :20].hex()
    if data.startswith(bytes.fromhex('00000001 0950')):
        # I - key frame
        with open('tmp.h264', 'wb') as f:
            f.write(data)
    elif data.startswith(bytes.fromhex('00000001 0930')):
        # P-frame}
        if i_frame_only:
            return None
        with open('tmp.h264', 'ab') as f:
            f.write(data)
    else:
        assert 0, f'Unexpected data {data[:20].hex()}'
    cap = cv2.VideoCapture('tmp.h264')
    image = None
    ret = True
    while ret:
        ret, frame = cap.read()
        if ret:
            image = frame #pygame.image.frombuffer(frame.tobytes(), frame.shape[1::-1], "BGR")
    cap.release()
    return image


def read_logfile(logfile, writer=None, add_time=True):
    nn_mask_stream = lookup_stream_id(logfile, 'oak.nn_mask')
    img_stream = lookup_stream_id(logfile, 'oak.color')
    pose2d_stream = lookup_stream_id(logfile, 'platform.pose2d')
    total_duration, total_dist = get_time_and_dist(logfile, 'platform.pose2d')

    with LogReader(logfile, only_stream_id=[nn_mask_stream, img_stream, pose2d_stream]) as log:
        img = np.zeros((1080, 1920, 3), dtype='uint8')
        dist = 0
        prev = [0, 0, 0]
        for timestamp, stream_id, data in log:
            mear_the_end = timestamp > total_duration - datetime.timedelta(seconds=10)
            if stream_id == nn_mask_stream:
                if img is None:
                    continue
                mask = deserialize(data)
                assert mask.shape == (120, 160), mask.shape
                mask[:60, :] = 0  # remove sky detections
                center_y, center_x = mask_center(mask)
                scale = 12  # 160 -> 640 -> 1920
                center_x *= scale
                center_y *= 9  # scale
                mask = cv2.resize(mask, (1920, 1080))
                height, width = mask.shape
                colored_mask = np.zeros((height, width, 3), dtype=np.uint8)
                colored_mask[mask == 1] = [0, 0, 255]
                overlay = cv2.addWeighted(img, 1, colored_mask, 0.7, 0)

                cross_length = 50
                thickness = 5
                cv2.line(overlay, (center_x - cross_length, center_y), (center_x + cross_length, center_y), (0, 255, 0),
                         thickness=thickness)
                cv2.line(overlay, (center_x, center_y - cross_length), (center_x, center_y + cross_length), (0, 255, 0),
                         thickness=thickness)

                # arrow
                dead = 10 * scale
                if center_x > width/2 + dead:
                    cv2.line(overlay, (center_x + cross_length, center_y),
                             (center_x + cross_length // 2, center_y - cross_length // 3), (0, 255, 0),
                             thickness=thickness)
                    cv2.line(overlay, (center_x + cross_length, center_y),
                             (center_x + cross_length // 2, center_y + cross_length // 3), (0, 255, 0),
                         thickness=thickness)
                elif center_x < width/2 - dead:
                    cv2.line(overlay, (center_x - cross_length, center_y),
                            (center_x - cross_length//2, center_y - cross_length//3), (0, 255, 0),
                             thickness=thickness)
                    cv2.line(overlay, (center_x - cross_length, center_y),
                            (center_x - cross_length//2, center_y + cross_length//3), (0, 255, 0),
                             thickness=thickness)

                if add_time:
                    x, y = 600, 100
                    thickness = 5
                    size = 5.0
                    # clip microseconds to miliseconds
                    s = str(timestamp)[:-3] + f' ({dist:.1f}m)'
                    cv2.putText(overlay, s, (x, y), cv2.FONT_HERSHEY_PLAIN,
                                size, (255, 0, 0), thickness=thickness)

                cv2.imshow("OAK-D Segmentation", overlay)
                if writer is not None:
                    writer.write(overlay)

                key = cv2.waitKey(1)
                if key == 0x20:
                    key = cv2.waitKey(0)
                if key == ord('s'):
                    cv2.imwrite('save_img.jpg', overlay)
                if key in [27, ord('q')]:
                    break

            elif stream_id == img_stream:
                img = read_h264_image(deserialize(data), i_frame_only=not mear_the_end)
            elif stream_id == pose2d_stream:
                pose2d = deserialize(data)
                dist += math.hypot((pose2d[0] - prev[0]) / 1000.0, (pose2d[1] - prev[1]) / 1000.0)
                prev = pose2d
            else:
                assert 0, stream_id  # unexpected stream
    if writer is not None:
        for i in range(20):
            writer.write(overlay)


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', nargs='+', help='recorded log file(s)')
    parser.add_argument('--create-video', help='filename of output video')
    args = parser.parse_args()

    if args.create_video is not None:
        fps = 20
        width, height = 1920, 1080
        writer = cv2.VideoWriter(args.create_video,
                                 cv2.VideoWriter_fourcc(*"mp4v"),
                                 fps,
                                 (width, height))
    else:
        writer = None  # no video writer

    for logfile in args.logfile:
        read_logfile(logfile, writer=writer)

    if writer is not None:
        writer.release()


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4 
