#!/usr/bin/python
"""
  Demo example how to read log file
"""
import pathlib
from datetime import timedelta

import cv2
import numpy as np

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize

from main import mask_center


def read_h264_image(data):
    assert data.startswith(bytes.fromhex('00000001 0950')) or data.startswith(bytes.fromhex('00000001 0930')), data[
                                                                                                               :20].hex()
    if data.startswith(bytes.fromhex('00000001 0950')):
        # I - key frame
        with open('tmp.h264', 'wb') as f:
            f.write(data)
    elif data.startswith(bytes.fromhex('00000001 0930')):
        # P-frame}
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


def read_logfile(logfile, video_filename=None):
    nn_mask_stream = lookup_stream_id(logfile, 'oak.nn_mask')
    img_stream = lookup_stream_id(logfile, 'oak.color')
    outfile = video_filename
    fps = 20
    width, height = 1920, 1080
    if outfile is not None:
        writer = cv2.VideoWriter(outfile,
                                 cv2.VideoWriter_fourcc(*"mp4v"),
                                 fps,
                                 (width, height))

    with LogReader(logfile, only_stream_id=[nn_mask_stream, img_stream]) as log:
#        img = np.zeros((480, 640, 3), dtype='uint8')
        img = np.zeros((1080, 1920, 3), dtype='uint8')
        for timestamp, stream_id, data in log:
            if stream_id == nn_mask_stream:
                mask = deserialize(data)
                center_y, center_x = mask_center(mask)
                scale = 12  # 160 -> 640 -> 1920
                center_x *= scale
                center_y *= 9  # scale
                #if timestamp > timedelta(seconds=6):
                    #return
#                mask = cv2.resize(mask, (640, 480))
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


                cv2.imshow("OAK-D Segmentation", overlay)
                if outfile is not None:
                    writer.write(overlay)

                key = cv2.waitKey(1)
                if key == 0x20:
                    key = cv2.waitKey(0)
                if key == ord('s'):
                    cv2.imwrite('save_img.jpg', overlay)
                if key in [27, ord('q')]:
                    break

            if stream_id == img_stream:
#                img = cv2.resize(read_h264_image(deserialize(data)), (640, 480))
                img = read_h264_image(deserialize(data))
    if outfile is not None:
        writer.release()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Extract data from logfile')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--create-video', help='filename of output video')
    args = parser.parse_args()

    read_logfile(args.logfile, video_filename=args.create_video)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4 

