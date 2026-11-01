import datetime
import tempfile
import unittest
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, call, patch

import av
import numpy as np
from osgar.lib.serialize import serialize
from osgar.logger import LogWriter

from dh2.viewer import Recording, depth_image, fit, layout, play


class ViewerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.path = Path(cls.directory.name) / 'test.log'
        codec = av.CodecContext.create('libx265', 'w')
        codec.width, codec.height = 64, 48
        codec.pix_fmt = 'yuv420p'
        codec.time_base = Fraction(1, 10)
        codec.options = {'x265-params': 'log-level=error:pools=none:frame-threads=1:keyint=5:min-keyint=5:'
                         'scenecut=0:bframes=0:repeat-headers=1'}
        packets = []
        for i in range(12):
            frame = av.VideoFrame.from_ndarray(np.full((48, 64, 3), i * 15, np.uint8), format='bgr24')
            frame.pts = i
            packets.extend(codec.encode(frame))
        packets.extend(codec.encode(None))
        with LogWriter(filename=str(cls.path)) as log:
            color = log.register('oak.color')
            depth = log.register('oak.depth')
            status = log.register('app.status')
            log.write(status, serialize('scanning'), dt=datetime.timedelta(seconds=0.9))
            log.write(depth, serialize(np.full((4, 6), 500)), dt=datetime.timedelta(seconds=0.95))
            for i, packet in enumerate(packets):
                sec = 1 + i / 10
                log.write(color, serialize(bytes(packet)), dt=datetime.timedelta(seconds=sec))
                log.write(depth, serialize(np.full((4, 6), 1000 + i * 100)),
                          dt=datetime.timedelta(seconds=sec + 0.02))

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def test_seek_matches_continuous_decode_and_can_go_backwards(self):
        with Recording(self.path) as recording:
            full = list(recording.frames())
            self.assertEqual(len(full), 12)
            self.assertGreater(len(recording.keys), 1)
            for start in (1.7, 1.05, 2.05, 0):
                seek = next(recording.frames(start))
                expected = next(frame for frame in full if frame[0] >= start)
                self.assertAlmostEqual(seek[0], expected[0])
                np.testing.assert_array_equal(seek[1], expected[1])
            self.assertEqual(list(recording.frames(100)), [])

    def test_synchronization_uses_preceding_sample_even_after_backward_seek(self):
        with Recording(self.path) as recording:
            self.assertEqual(recording.latest('oak.depth', 0.8), (None, None))
            stamp, depth = recording.latest('oak.depth', 1)
            self.assertEqual(stamp, 0.95)
            self.assertTrue((depth == 500).all())
            recording.latest('oak.depth', 2)
            stamp, depth = recording.latest('oak.depth', 1.1)
            self.assertEqual(stamp, 1.02)
            self.assertTrue((depth == 1000).all())
            self.assertEqual(recording.latest('missing', 1), (None, None))

    def test_delayed_frames_without_time_base_keep_packet_timestamps(self):
        with Recording(self.path) as recording:
            expected = list(recording.frames())
            decoder = av.CodecContext.create('hevc', 'r')
            decoder.thread_count = 2
            flushed = []
            pending = []

            def decode(packet):
                pending.extend(decoder.decode(packet))
                if packet is None:
                    flushed.extend(pending)
                    return [SimpleNamespace(pts=f.pts, time_base=None, to_ndarray=f.to_ndarray) for f in pending]
                frames = pending[:-2]
                del pending[:-2]
                return frames

            proxy = MagicMock()
            proxy.decode.side_effect = decode
            with patch('dh2.viewer.av.CodecContext') as context:
                context.create.return_value = proxy
                actual = list(recording.frames())
            self.assertTrue(flushed)
            self.assertEqual([t for t, _ in actual], [t for t, _ in expected])
            for (_, image), (_, reference) in zip(actual, expected):
                np.testing.assert_array_equal(image, reference)

    def test_zero_and_invalid_depth_are_black(self):
        color = depth_image(np.array([[0, np.nan, -1, 300, 5000, 9000]]), 0.3, 5)
        self.assertFalse(color[0, :3].any())
        self.assertTrue(color[0, 3:].any())
        np.testing.assert_array_equal(color[0, 4], color[0, 5])

    def test_layout_and_aspect_ratio(self):
        image = fit(np.full((10, 10, 3), 255, np.uint8), 20, 10)
        self.assertFalse(image[:, :5].any())
        self.assertTrue((image[:, 5:15] == 255).all())
        with Recording(self.path) as recording:
            frame = next(recording.frames(1.7))
            image = layout(recording, frame, paused=True)
            self.assertEqual(image.shape, (720, 1280, 3))

    def test_camera_free_log_has_clear_error(self):
        path = Path(self.directory.name) / 'stop.log'
        with LogWriter(filename=str(path)) as log:
            stream = log.register('platform.emergency_stop')
            log.write(stream, serialize(True))
        with self.assertRaisesRegex(ValueError, 'no HEVC color keyframes'):
            with Recording(path):
                pass

    def test_playback_step_seek_and_slider_callbacks(self):
        from argparse import Namespace

        args = Namespace(start=1.7, pause=True, speed=1, near=0.3, far=5)
        callbacks = []

        def create_slider(name, window, value, maximum, callback):
            callbacks.append(callback)
            callback(0)

        with Recording(self.path) as recording, patch('dh2.viewer.cv2') as gui, patch('dh2.viewer.layout'):
            gui.createTrackbar.side_effect = create_slider
            gui.setTrackbarPos.side_effect = lambda name, window, value: callbacks[0](value)
            gui.getWindowProperty.return_value = 1
            gui.waitKeyEx.side_effect = [ord('.'), ord('a'), ord('n')]
            with patch.object(recording, 'frames', wraps=recording.frames) as frames:
                self.assertEqual(play(recording, args, 0, 2), 1)
                self.assertEqual(frames.call_args_list, [call(1.7), call(0)])


if __name__ == '__main__':
    unittest.main()
