import unittest
from unittest.mock import MagicMock, call

from doctor import Doctor, VIDEO_OUTPUT_ROOT


class DoctorTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        ref_h265_data = bytes.fromhex('00000001 460150') + b'some H265 binary data'  # must be I-frame
        doctor = Doctor(bus=bus, config={})
        doctor.on_scanning_person(True)
        doctor.on_h265_video(ref_h265_data)
        doctor.on_scanning_person(False)

        with open(VIDEO_OUTPUT_ROOT / 'video1.h265', 'rb') as f:
            content = f.read()
        self.assertEqual(content, ref_h265_data)


if __name__ == '__main__':
    unittest.main()
