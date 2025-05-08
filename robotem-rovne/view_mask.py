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


def read_logfile(logfile):
    nn_mask_stream = lookup_stream_id(logfile, 'oak.nn_mask')
    img_stream = lookup_stream_id(logfile, 'oak.color')
    with LogReader(logfile, only_stream_id=[nn_mask_stream, img_stream]) as log:
        for timestamp, stream_id, data in log:
            if stream_id == nn_mask_stream:
                mask = deserialize(data)
                #if timestamp > timedelta(seconds=6):
                    #return
                img = np.zeros((480, 640, 3), dtype='uint8')
                mask = cv2.resize(mask, (640, 480))
                height, width = mask.shape
                colored_mask = np.zeros((height, width, 3), dtype=np.uint8)
                colored_mask[mask == 1] = [0, 0, 255]
                overlay = cv2.addWeighted(img, 1, colored_mask, 0.7, 0)

                cv2.imshow("OAK-D Segmentation", overlay)
                key = cv2.waitKey(100)
                if key in [27, ord('q')]:
                    break


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Extract data from logfile')
    parser.add_argument('logfile', help='recorded log file')
    args = parser.parse_args()

    read_logfile(args.logfile)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4 

