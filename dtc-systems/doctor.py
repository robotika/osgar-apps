"""
  Module for medical evaluation - name "doctor" is in memory of GLB (that time to take care of other modules)
"""
from pathlib import Path

import cv2
import wave

from osgar.node import Node


AUDIO_OUTPUT_ROOT = Path(__file__).parent / 'dtc_report' / 'audio'
VIDEO_OUTPUT_ROOT = Path(__file__).parent / 'dtc_report' / 'video'


class Doctor(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('report')
        self.is_scanning = False
        self.report_index = 0
        self.wav_fd = None
        self.h265_fd = None
        self.key_frame_detected = False
        AUDIO_OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
        VIDEO_OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
        self.verbose = False  # TODO move to Node default

    def on_report_latlon(self, data):
        """
        initial lat, lon position of the report
        """
        pass

    def on_scanning_person(self, data):
        """
        Boolean trigger if the robot is in stationary scanning mode
        """
        if data and not self.is_scanning:
            # open files for recording video and audio
            self.report_index += 1
            assert self.wav_fd is None
            self.wav_fd = wave.open(str(AUDIO_OUTPUT_ROOT / f'audio{self.report_index}.wav'), 'wb')
            # TODO get audio_info directly from source Node
            channels = 1
            sample_width = 2
            rate = 44100
            self.wav_fd.setnchannels(channels)
            self.wav_fd.setsampwidth(sample_width)
            self.wav_fd.setframerate(rate)

            assert self.h265_fd is None
            self.h265_fd = open(VIDEO_OUTPUT_ROOT / f'video{self.report_index}.h265', 'wb')
            self.key_frame_detected = False

        if self.is_scanning and not data:
            assert self.wav_fd is not None
            self.wav_fd.close()
            self.wav_fd = None
            assert self.h265_fd is not None
            self.h265_fd.close()
            self.h265_fd = None
            if self.verbose:
                cap = cv2.VideoCapture(str(VIDEO_OUTPUT_ROOT / f'video{self.report_index}.h265'))
                while True:
                    ret, frame = cap.read()
                    if ret == 0:
                        break
                    cv2.imshow(f'video{self.report_index}.h265', frame)
                    cv2.waitKey(100)
                cap.release()

        self.is_scanning = data

    def on_audio(self, data):
        """
        Collect audio sample during scanning period
        """
        if self.is_scanning:
            assert self.wav_fd is not None
            self.wav_fd.writeframes(data)

    def on_h265_video(self, data):
        """
        Collect H.265 data during scanning period
        """
        if self.is_scanning:
            assert self.h265_fd is not None
            if not self.key_frame_detected:
                self.key_frame_detected =  (data.startswith(bytes.fromhex('00000001 0950')) or
                                            data.startswith(bytes.fromhex('00000001 460150')))
            if self.key_frame_detected:
                self.h265_fd.write(data)



if __name__ == "__main__":
    pass
