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
        doctor.last_location = {'lat': 32.6570764, 'lon': -83.7562508}
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
        doctor.last_location = {'lat': 32.6570764, 'lon': -83.7562508}
        doctor.on_scanning_person(True)
        doctor.on_audio(audio_data)
        doctor.on_scanning_person(False)
        doctor.on_scanning_person(True)
        doctor.on_audio(audio_data)
        doctor.on_scanning_person(False)

    def test_fb_report(self):
        bus = MagicMock()
        doctor = Doctor(bus=bus, config={})
        doctor.last_location = {'lat': 32.6570764, 'lon': -83.7562508}
        fb_report = {'Head': 'Normal',
             'Heart Rate': 0,
             'Lower Extermities': 'Normal',
             'Motor': 'Absent',
             'Ocular': 'Open',
             'Respiratory Distress': 'Absent',
             'Respiratory Rate': 13,
             'Severe Hemorrhage': 'Absent',
             'Torso': 'Normal',
             'Upper Extermities': 'Normal',
             'Verbal': '<TODO>'}
        doctor.report_index = 1
        doctor.publish_report(fb_report, audio_pair=[False, ''])
        last = bus.mock_calls[-1]
        self.assertEqual(last.args[0], 'report')
        report = last.args[1]
        self.assertEqual(report['severe_hemorrhage']['value'], 0)
        self.assertEqual(report['respiratory_distress']['value'], 0)
        self.assertEqual(report['hr']['value'], 0)
        self.assertEqual(report['rr']['value'], 13)

        self.assertEqual(report['trauma_head'], 0)
        self.assertEqual(report['trauma_torso'], 0)
        self.assertEqual(report['trauma_lower_ext'], 0)
        self.assertEqual(report['trauma_upper_ext'], 0)

        self.assertEqual(report['alertness_ocular']['value'], 0)
        self.assertEqual(report['alertness_motor']['value'], 2)


if __name__ == '__main__':
    unittest.main()
