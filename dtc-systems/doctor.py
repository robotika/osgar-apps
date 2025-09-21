"""
  Module for medical evaluation - name "doctor" is in memory of GLB (that time to take care of other modules)
"""
from pathlib import Path

import cv2
import wave
from ultralytics import YOLO

from osgar.node import Node
from wav2txt import is_coherent_speech
from report import DTCReport, pack_data


AUDIO_OUTPUT_ROOT = Path(__file__).parent / 'dtc_report' / 'audio'
VIDEO_OUTPUT_ROOT = Path(__file__).parent / 'dtc_report' / 'video'

DTC_QUERY_SOUND = 'can_you_hear_me'


class Doctor(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('report', 'lora_report', 'audio_analysis', 'play_sound')
        self.system_name = config.get('env', {}).get('OSGAR_LOGS_PREFIX', 'M01')
        self.is_scanning = False
        self.is_playing = False
        self.report_index = 0
        self.wav_fd = None
        self.h265_fd = None
        self.key_frame_detected = False
        AUDIO_OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
        VIDEO_OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
        self.onnx_model = YOLO('models/yolo11n-pose.onnx')  # parametrize?
        self.verbose = False  # TODO move to Node default
        self.last_location = None

    def publish_report(self):
        assert self.last_location is not None
        r = DTCReport(self.system_name, self.last_location['lat'], self.last_location['lon'])
        r.severe_hemorrhage = 0  # absent
        r.respiratory_distress = 0  # absent
        r.hr = 70
        r.rr = 15
        assert self.report_index > 0, self.report_index
        r.casualty_id = self.report_index
        self.publish('lora_report', pack_data(r) + b'\n')
        self.publish('report', r.tojson())

    def on_report_latlon(self, data):
        """
        initial lat, lon position of the report
        """
        self.last_location = data.copy()

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
            self.publish('play_sound', DTC_QUERY_SOUND)
            self.is_playing = True

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
                    results = self.onnx_model(frame)
#                    print(results[0].keypoints)
                    kpts = results[0].keypoints.xy.detach().cpu().numpy()[0]
                    pose_w_id = results[0].plot()
                    cv2.imshow(f'video{self.report_index}.h265', pose_w_id)  #frame)
                    cv2.waitKey(100)
                cap.release()
            is_coherent, text = is_coherent_speech(str(AUDIO_OUTPUT_ROOT / f'audio{self.report_index}.wav'))
            self.publish('audio_analysis', [is_coherent, text])
            self.publish_report()

        self.is_scanning = data

    def on_audio(self, data):
        """
        Collect audio sample during scanning period
        """
        if self.is_scanning and not self.is_playing:
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

    def on_playing(self, data):
        name, status = data
        if name == DTC_QUERY_SOUND:
            self.is_playing = status
        # ... but maybe we would like to track also playing other sounds??


if __name__ == "__main__":
    pass
