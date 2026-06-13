import unittest
from unittest.mock import MagicMock, call

import cv2

from follow_apriltag import AprilTag


class AprilTagTest(unittest.TestCase):
    def test_detect_april_tags(self):
        config = {}
        bus = MagicMock()
        node = AprilTag(config, bus)

        image = cv2.imread("april-tags.jpg", 0)
        tags = node.detect_april_tags(image)
        self.assertEqual(tags[0], [3, 4, 1, 2])
        self.assertEqual(len(tags[1][0]), 4)
        self.assertEqual(tags[1][0][0], [664, 684])


if __name__ == '__main__':
    unittest.main()
