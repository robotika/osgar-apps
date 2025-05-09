import unittest

import numpy as np

from view_mask import mask_center


class ViewMaskTest(unittest.TestCase):

    def test_mask_center(self):
        mask = np.zeros((120, 160), dtype='uint8')
        self.assertEqual(mask_center(mask), (60, 80))

        mask[10, 20] = 1
        self.assertEqual(mask_center(mask), (10, 20))

        mask[20, 20] = 1
        self.assertEqual(mask_center(mask), (15, 20))


if __name__ == "__main__":
    unittest.main()

# vim: expandtab sw=4 ts=4
