import unittest
from unittest.mock import MagicMock, call

import numpy as np

from doctor import Doctor, VIDEO_OUTPUT_ROOT, AUDIO_OUTPUT_ROOT


class DoctorTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        ref_h265_data = bytes.fromhex('00000001 460150') + b'some H265 binary data'  # must be I-frame
        audio_data = np.zeros(100, dtype=np.uint16)
        doctor = Doctor(bus=bus, config={})
        doctor.on_scanning_person(True)
        doctor.on_h265_video(ref_h265_data)
        doctor.on_audio(audio_data)
        doctor.on_scanning_person(False)

        with open(VIDEO_OUTPUT_ROOT / 'video1.h265', 'rb') as f:
            content = f.read()
        self.assertEqual(content, ref_h265_data)

        with open(AUDIO_OUTPUT_ROOT / 'audio1.wav', 'rb') as f:
            content = f.read()
        self.assertEqual(content[:4], b'RIFF')

    def test_audio_bug(self):
        bus = MagicMock()
        audio_data = np.zeros(100, dtype=np.uint16)
        doctor = Doctor(bus=bus, config={})
        doctor.on_scanning_person(True)
        doctor.on_audio(audio_data)
        doctor.on_scanning_person(False)
        doctor.on_scanning_person(True)
        doctor.on_audio(audio_data)
        doctor.on_scanning_person(False)


if __name__ == '__main__':
    unittest.main()
