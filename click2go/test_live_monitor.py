"""Monitor isolation, growing logs, latest-value state and network budgets."""

import json
import os
import tempfile
import threading
import time
import unittest
from datetime import datetime, timedelta, timezone
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import urlopen

import cv2
import numpy as np
from ground_projection import GroundProjection
from live_monitor import ByteBudget, DirectorySource, LogDirectory, Snapshots, State, follow, follow_directory, handler
from osgar.lib.serialize import serialize
from osgar.logger import LogWriter, format_header, format_packet
from test_ground_projection import synthetic_calibration


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError('Monitor did not catch up')


class MonitorTest(unittest.TestCase):
    def test_budget_drops_instead_of_waiting(self):
        now = [0.0]
        budget = ByteBudget(100.0, burst=200.0, clock=lambda: now[0])
        self.assertTrue(budget.take(150))
        self.assertFalse(budget.take(100))
        now[0] = 0.5
        self.assertTrue(budget.take(100))
        self.assertFalse(budget.take(1))
        now[0] = 100.0
        self.assertFalse(budget.take(201))
        self.assertFalse(budget.take(180, reserve=30))
        self.assertTrue(budget.take(180))

    def test_latest_values_and_frozen_log_become_old(self):
        state = State('example.log')
        state.start_utc = 100.0
        state.log_time = 10.0
        state.ingest(8.0, 'platform.emergency_stop', False)
        state.ingest(9.0, 'app.desired_steering', [100, 2000])
        state.ingest(10.0, 'app.desired_steering', [0, 0])
        snapshot = state.snapshot(now=120.0)
        self.assertEqual(snapshot['log_age_s'], 10.0)
        self.assertEqual(snapshot['samples']['platform.emergency_stop']['age_s'], 12.0)
        self.assertEqual(snapshot['samples']['app.desired_steering']['value'], [0, 0])
        json.dumps(snapshot, allow_nan=False)

    def test_partial_append_reconnect_and_read_only(self):
        with tempfile.TemporaryDirectory() as directory:
            original, partial = Path(directory) / 'source.log', Path(directory) / 'growing.log'
            with LogWriter(filename=str(original)) as log:
                steering = log.register('app.desired_steering', dt=timedelta())
                can = log.register('platform.can', dt=timedelta())
                log.write(steering, serialize([100, 2000]), dt=timedelta(seconds=1))
                log.write(can, serialize([123, b'not telemetry', 1]), dt=timedelta(seconds=1.1))
                log.write(steering, serialize([0, 0]), dt=timedelta(seconds=2))
            content = original.read_bytes()
            partial.write_bytes(content[:-3])
            state, stop = State(partial), threading.Event()
            thread = threading.Thread(target=follow, args=(state, stop, 0.0), daemon=True)
            thread.start()
            try:
                wait_for(lambda: 'app.desired_steering' in state.snapshot()['samples'])
                self.assertEqual(state.snapshot()['samples']['app.desired_steering']['value'], [100, 2000])
                with partial.open('ab') as stream:
                    stream.write(content[-3:])
                wait_for(lambda: state.snapshot()['samples']['app.desired_steering']['value'] == [0, 0])
                self.assertNotIn('platform.can', state.snapshot()['samples'])
            finally:
                stop.set()
                thread.join(2.0)
            self.assertFalse(thread.is_alive())
            self.assertEqual(partial.read_bytes(), content)
            # A new monitor session reconstructs the latest state, not a network queue.
            again, stop = State(partial), threading.Event()
            thread = threading.Thread(target=follow, args=(again, stop, 0.0), daemon=True)
            thread.start()
            try:
                wait_for(lambda: again.snapshot()['samples'].get('app.desired_steering', {}).get('value') == [0, 0])
            finally:
                stop.set()
                thread.join(2.0)

    def test_snapshot_is_bounded_and_optional(self):
        pixels = np.zeros((1080, 1920, 3), np.uint8)
        pixels[:, :, 1] = 120
        ok, jpeg = cv2.imencode('.jpg', pixels)
        self.assertTrue(ok)
        snapshots = Snapshots(5.0)
        image = snapshots.image(jpeg.tobytes())
        decoded = cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_COLOR)
        self.assertLessEqual(decoded.shape[1], 640)
        self.assertLessEqual(len(image), 16384)
        self.assertEqual(snapshots.source_size, [1920, 1080])
        self.assertIsNone(snapshots.image(jpeg.tobytes()))

    def test_projection_is_read_only_and_bound_to_displayed_frame(self):
        state = State('example.log')
        state.rgb_seq, state.rgb_size, state.rgb_stamp = 3, [1920, 1080], 12.0
        projection = GroundProjection(synthetic_calibration())
        with ThreadingHTTPServer(('127.0.0.1', 0), handler(state, ByteBudget(8192.0), projection)) as server:
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            url = f'http://127.0.0.1:{server.server_port}'
            query = f'/project?session={state.session_id}&seq=3&x=960&y=1040'
            try:
                with urlopen(url + query, timeout=2.0) as response:
                    result = json.load(response)
                    self.assertEqual(result['target_robot_m'], [1.1, 0.0, 0.0])
                    self.assertEqual(result['log_time_s'], 12.0)
                with urlopen(url + '/state', timeout=2.0) as response:
                    self.assertTrue(json.load(response)['projection_enabled'])
                for suffix, code in (
                    (query.replace('seq=3', 'seq=2'), 409),
                    (query.replace(state.session_id, 'old-session'), 409),
                    (query.replace('y=1040', 'y=540'), 400),
                    (query.replace('x=960', 'x=nan'), 400),
                ):
                    with self.subTest(suffix=suffix), self.assertRaises(HTTPError) as error:
                        urlopen(url + suffix, timeout=2.0)
                    self.assertEqual(error.exception.code, code)
                self.assertEqual(state.snapshot()['samples'], {})
            finally:
                server.shutdown()
                thread.join(2.0)

    def test_http_has_no_control_and_refuses_over_budget(self):
        state = State('example.log')
        state.ingest(1.0, 'platform.pose2d', [0, 0, 0])
        budget = ByteBudget(8192.0)
        with ThreadingHTTPServer(('127.0.0.1', 0), handler(state, budget)) as server:
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            url = f'http://127.0.0.1:{server.server_port}'
            try:
                with urlopen(url + '/state', timeout=2.0) as response:
                    self.assertEqual(json.load(response)['samples']['platform.pose2d']['value'], [0, 0, 0])
                with urlopen(url, timeout=2.0) as response:
                    self.assertIn(b'No robot controls', response.read())
                with self.assertRaises(HTTPError) as error:
                    urlopen(url + '/arm', timeout=2.0)
                self.assertEqual(error.exception.code, 404)
                with self.assertRaises(HTTPError) as error:
                    urlopen(url + '/project?x=1&y=2', timeout=2.0)
                self.assertEqual(error.exception.code, 404)
                with self.assertRaises(HTTPError) as error:
                    urlopen(url + '/state', data=b'arm', timeout=2.0)
                self.assertEqual(error.exception.code, 501)
                with patch.object(budget, 'take', return_value=False):
                    with self.assertRaises(HTTPError) as error:
                        urlopen(url + '/state', timeout=2.0)
                    self.assertEqual(error.exception.code, 429)
            finally:
                server.shutdown()
                thread.join(2.0)


class DirectoryMonitorTest(unittest.TestCase):
    def recording(self, path, second, speed):
        with LogWriter(filename=str(path)) as log:
            channel = log.register('app.desired_steering', dt=timedelta())
            log.write(channel, serialize([speed, 0]), dt=timedelta(seconds=1))
        content = path.read_bytes()
        header = b''.join(format_header(datetime(2026, 1, 1, 0, 0, second, tzinfo=timezone.utc)))
        path.write_bytes(header + content[16:])
        return path.read_bytes()

    def test_selection_uses_valid_header_time_not_name_or_mtime(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            older, newer = root / 'z-old.log', root / 'a-new.log'
            self.recording(older, 1, 10)
            self.recording(newer, 2, 20)
            os.utime(older, (time.time() + 3600, time.time() + 3600))
            (root / 'garbage.log').write_bytes(b'garbage that is not a log')
            (root / 'partial.log').write_bytes(b'ALog')
            selector = LogDirectory(root)
            self.assertEqual(selector.newest()[0], newer)
            # Reopening a newer log's partial header makes it selectable on the next scan.
            newest = root / 'partial.log'
            self.recording(newest, 3, 30)
            self.assertEqual(selector.newest()[0], newest)
            self.assertEqual(LogDirectory(root, 'z-*.log').newest()[0], older)
            with self.assertRaises(ValueError):
                LogDirectory(root, '../*.log')

    def test_wait_switch_reset_and_older_append_does_not_steal_selection(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, stop = DirectorySource(root), threading.Event()
            thread = threading.Thread(target=follow_directory, args=(source, stop, 0.0, '*.log', 0.02), daemon=True)
            thread.start()
            try:
                self.assertTrue(source.snapshot()['waiting_for_log'])
                # Do not switch to an incomplete header.
                (root / 'new.log').write_bytes(b'ALog')
                self.recording(root / 'old.log', 1, 10)
                wait_for(lambda: source.snapshot()['samples'].get('app.desired_steering', {}).get('value') == [10, 0])
                old_state = source.current
                old_session = source.snapshot()['session_id']
                old_state.ingest(1.0, 'platform.joint_angle', 11)
                with old_state.lock:
                    old_state.rgb = b'old image'
                    old_state.rgb_seq = 1
                self.recording(root / 'new.log', 2, 20)
                wait_for(lambda: source.snapshot()['samples'].get('app.desired_steering', {}).get('value') == [20, 0])
                snapshot = source.snapshot()
                self.assertNotEqual(snapshot['session_id'], old_session)
                self.assertEqual(snapshot['file'], 'new.log')
                self.assertEqual(snapshot['selection_mode'], 'directory')
                self.assertNotIn('platform.joint_angle', snapshot['samples'])
                self.assertIsNone(source.get_rgb()[0])
                old_state.ingest(2.0, 'app.desired_steering', [999, 0])
                with (root / 'old.log').open('ab') as stream:
                    stream.write(b''.join(format_packet(1, serialize([99, 0]), timedelta(seconds=3))))
                time.sleep(0.08)
                self.assertEqual(source.snapshot()['file'], 'new.log')
                self.assertEqual(source.snapshot()['samples']['app.desired_steering']['value'], [20, 0])
            finally:
                stop.set()
                thread.join(3.0)
            self.assertFalse(thread.is_alive())

    def test_replacement_same_filename_starts_new_session(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / 'same.log'
            self.recording(path, 1, 10)
            source, stop = DirectorySource(root), threading.Event()
            thread = threading.Thread(target=follow_directory, args=(source, stop, 0.0, '*.log', 0.02), daemon=True)
            thread.start()
            try:
                wait_for(lambda: source.snapshot()['samples'].get('app.desired_steering', {}).get('value') == [10, 0])
                first = source.snapshot()['session_id']
                replacement = root / 'replacement.tmp'
                self.recording(replacement, 2, 20)
                replacement.replace(path)
                wait_for(lambda: source.snapshot()['samples'].get('app.desired_steering', {}).get('value') == [20, 0])
                self.assertNotEqual(first, source.snapshot()['session_id'])
            finally:
                stop.set()
                thread.join(3.0)
