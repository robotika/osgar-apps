"""Optional read-only log monitor. Run separately from osgar.record/control.

HTTP binds to loopback only; view through an SSH tunnel. Only latest state is
retained. No control bus, CAN connection, recorder subprocess or actuator input.
"""

import argparse
import datetime
import json
import math
import re
import struct
import threading
import time
import uuid
from ast import literal_eval
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from ground_projection import GroundProjection
from osgar.lib.serialize import deserialize
from osgar.logger import MAGIC, LogReader

# Only explicitly selected telemetry is deserialized; camera data stays on the robot.
TELEMETRY = {
    'app.desired_steering',
    'platform.pose2d',
    'platform.emergency_stop',
    'platform.bumpers_front',
    'platform.bumpers_rear',
    'platform.joint_angle',
    'app.status',
}
START_CODE = re.compile(b'\x00\x00\x00?\x01')


def compact(value, depth=0):
    """Bound telemetry size and normalize arrays/nonfinite values for JSON."""
    if depth > 4:
        return '...'
    if hasattr(value, 'tolist'):
        value = value.tolist()
    if isinstance(value, dict):
        return {str(k)[:80]: compact(v, depth + 1) for k, v in list(value.items())[:32]}
    if isinstance(value, (list, tuple)):
        return [compact(v, depth + 1) for v in value[:64]]
    if isinstance(value, str):
        return value[:256]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if value is None or isinstance(value, (bool, int, float)):
        return value
    return str(value)[:80]


class ByteBudget:
    """Global response-body token bucket; never waits or queues a response."""

    def __init__(self, rate, burst=None, clock=time.monotonic):
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('Payload rate must be positive')
        self.rate, self.capacity = rate, max(32768, rate * 2) if burst is None else burst
        self.clock, self.updated = clock, clock()
        self.tokens = self.capacity
        self.lock = threading.Lock()

    def take(self, size, reserve=0):
        with self.lock:
            now = self.clock()
            self.tokens = min(self.capacity, self.tokens + max(0, now - self.updated) * self.rate)
            self.updated = now
            if self.tokens < size + reserve:
                return False
            self.tokens -= size
            return True


class State:
    def __init__(self, path):
        self.path = Path(path) if path is not None else None
        self.session_id = uuid.uuid4().hex
        self.lock = threading.Lock()
        self.start_utc = None
        self.log_time = None
        self.last_read = None
        self.catching_up = True
        self.error = None
        self.samples = {}
        self.rgb = None
        self.rgb_stamp = None
        self.rgb_size = None
        self.rgb_seq = 0
        self.rgb_error = None
        self.bytes_read = 0
        self.file_size = 0

    def get_rgb(self):
        with self.lock:
            return self.rgb, f'"{self.session_id}:{self.rgb_seq}"'

    def ingest(self, stamp, name, value):
        value = compact(value)
        with self.lock:
            self.samples[name] = (stamp, value)

    def snapshot(self, now=None):
        now = time.time() if now is None else now
        with self.lock:
            elapsed = None if self.start_utc is None else max(0, now - self.start_utc)
            reference = max(elapsed or 0, self.log_time or 0)
            return dict(
                protocol=1,
                file=self.path.name if self.path is not None else None,
                session_id=self.session_id,
                waiting_for_log=self.path is None,
                selection_mode='file',
                start_utc=self.start_utc,
                log_time_s=self.log_time,
                log_age_s=None if self.log_time is None else max(0, reference - self.log_time),
                log_idle_s=None if self.last_read is None else max(0, time.monotonic() - self.last_read),
                catching_up=self.catching_up,
                error=self.error,
                bytes_read=self.bytes_read,
                file_size=self.file_size,
                rgb_seq=self.rgb_seq,
                rgb_size=self.rgb_size,
                rgb_age_s=None if self.rgb_stamp is None else max(0, reference - self.rgb_stamp),
                rgb_error=self.rgb_error,
                samples={
                    name: dict(log_time_s=t, age_s=max(0, reference - t), value=value)
                    for name, (t, value) in self.samples.items()
                },
            )


class LogDirectory:
    """Select valid .log headers by recording start time, not recent writes.

    A touch/rsync/append to an older log must not steal selection from a newer
    recording. Cache headers until file metadata changes; discovery is shallow.
    """

    def __init__(self, root, pattern='*.log'):
        self.root, self.pattern = Path(root), pattern
        if '/' in pattern or '\\' in pattern or not pattern:
            raise ValueError('Log pattern must be a filename glob without directories')
        self.cache = {}

    def newest(self):
        candidates, seen = [], set()
        for path in self.root.glob(self.pattern):
            try:
                stat = path.stat()
                if not path.is_file():
                    continue
                signature = (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)
                seen.add(path)
                cached = self.cache.get(path)
                if cached is None or cached[0] != signature:
                    with path.open('rb') as stream:
                        header = stream.read(16)
                    timestamp = None
                    if len(header) == 16 and header[:4] == MAGIC:
                        try:
                            timestamp = datetime.datetime(
                                *struct.unpack('HBBBBBI', header[4:]), datetime.timezone.utc
                            ).timestamp()
                        except (ValueError, struct.error):
                            pass
                    self.cache[path] = (signature, timestamp)
                timestamp = self.cache[path][1]
                if timestamp is not None:
                    candidates.append((timestamp, path.name, path, signature))
            except (FileNotFoundError, PermissionError):
                continue
        self.cache = {path: cached for path, cached in self.cache.items() if path in seen}
        if not candidates:
            return None
        timestamp, _, path, signature = max(candidates)
        return path, (*signature[:2], timestamp), signature[2]


class DirectorySource:
    """Publish one isolated reader session; never mix values across recordings."""

    def __init__(self, root):
        self.root = Path(root)
        self.lock = threading.Lock()
        self.current = State(None)
        self.error = None

    def snapshot(self, now=None):
        with self.lock:
            current, error = self.current, self.error
        result = current.snapshot(now)
        result.update(selection_mode='directory', directory=str(self.root))
        if error is not None:
            result['error'] = error
        return result

    def get_rgb(self):
        with self.lock:
            current = self.current
        return current.get_rgb()


def follow_directory(source, stop, rgb_interval=5.0, pattern='*.log', poll_interval=0.5):
    selector = LogDirectory(source.root, pattern)
    worker = worker_stop = None
    selected = None
    try:
        while not stop.is_set():
            candidate = selector.newest()
            identity = None if candidate is None else candidate[:2]
            with source.lock:
                current = source.current
            with current.lock:
                consumed = current.bytes_read
            truncated = candidate is not None and identity == selected and candidate[2] < consumed
            if identity != selected or truncated:
                if worker_stop is not None:
                    worker_stop.set()
                    worker.join(2.0)
                    if worker.is_alive():
                        raise RuntimeError('Previous log reader has not stopped')
                current = State(None if candidate is None else candidate[0])
                with source.lock:
                    source.current, source.error = current, None
                selected = identity
                worker = worker_stop = None
                if candidate is not None:
                    worker_stop = threading.Event()
                    worker = threading.Thread(target=follow, args=(current, worker_stop, rgb_interval), daemon=True)
                    worker.start()
            if stop.wait(poll_interval):
                break
    except Exception as error:
        with source.lock:
            source.error = f'{type(error).__name__}: {error}'[:256]
    finally:
        if worker_stop is not None:
            worker_stop.set()
            worker.join(2.0)


class Snapshots:
    """Decode only independent HEVC random-access pictures, at bounded cadence.

    No inter-frame video is sent. Parameter sets are collected while reading;
    no full frame history is kept. JPEG source frames are also supported.
    """

    def __init__(self, interval):
        self.interval = interval
        self.last_attempt = -math.inf
        self.parameters = {}
        self.source_size = None

    def image(self, data):
        starts = [m.start() for m in START_CODE.finditer(data)]
        units = [
            (data[START_CODE.match(data, a).end()] >> 1 & 63, data[a:b])
            for a, b in zip(starts, starts[1:] + [len(data)])
            if START_CODE.match(data, a).end() < b
        ]
        for kind, unit in units:
            if kind in (32, 33, 34):
                self.parameters[kind] = unit
        if time.monotonic() - self.last_attempt < self.interval:
            return None
        is_jpeg = data.startswith(b'\xff\xd8')
        if not is_jpeg and (not any(16 <= kind <= 21 for kind, _ in units) or len(self.parameters) != 3):
            return None
        self.last_attempt = time.monotonic()
        import cv2
        import numpy as np

        if is_jpeg:
            frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        else:
            import av

            decoder = av.CodecContext.create('hevc', 'r')
            decoder.thread_count = 1
            payload = b''.join(self.parameters[k] for k in sorted(self.parameters))
            payload += b''.join(unit for kind, unit in units if 16 <= kind <= 21)
            frames = decoder.decode(av.Packet(payload)) + decoder.decode(None)
            frame = frames[-1].to_ndarray(format='bgr24') if frames else None
        if frame is None:
            return None
        self.source_size = [frame.shape[1], frame.shape[0]]
        for width, quality in ((640, 50), (480, 40), (320, 30)):
            scale = min(1.0, width / frame.shape[1])
            small = cv2.resize(frame, (round(frame.shape[1] * scale), round(frame.shape[0] * scale)))
            ok, encoded = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if ok and encoded.size <= 16384:
                return encoded.tobytes()
        return None


def follow(state, stop, rgb_interval=5.0):
    """Disk reader cannot wait for a network client; LogReader handles split writes."""

    class Reader(LogReader):
        def _read(self, size):
            value = self.f.read(size)
            while len(value) < size and not stop.wait(0.1):
                value += self.f.read(size - len(value))
            return value

    try:
        while state.path.stat().st_size < 16:
            if stop.wait(0.1):
                return
        names = {}
        images = Snapshots(rgb_interval) if rgb_interval > 0 else None
        pending_rgb = None
        with Reader(str(state.path), follow=True) as reader:
            with state.lock:
                state.start_utc = reader.start_time.timestamp()
            for stamp, channel, raw in reader:
                if stop.is_set():
                    break
                t = stamp.total_seconds()
                offset, size = reader.f.tell(), state.path.stat().st_size
                catching_up = size - offset > 65536
                with state.lock:
                    state.log_time, state.last_read = t, time.monotonic()
                    state.bytes_read, state.file_size, state.catching_up = offset, size, catching_up
                if not catching_up and pending_rgb is not None:
                    rgb_time, rgb_data = pending_rgb
                    pending_rgb = None
                    try:
                        image = images.image(rgb_data)
                        if image is not None:
                            with state.lock:
                                state.rgb, state.rgb_stamp = image, rgb_time
                                state.rgb_size = images.source_size
                                state.rgb_seq += 1
                                state.rgb_error = None
                    except Exception as error:
                        with state.lock:
                            state.rgb_error = str(error)[:160]
                if channel == 0:
                    try:
                        metadata = literal_eval(raw.decode('ascii'))
                    except (ValueError, SyntaxError, UnicodeError):
                        continue
                    if isinstance(metadata, dict) and 'names' in metadata:
                        names = {i + 1: name for i, name in enumerate(metadata['names'])}
                    continue
                name = names.get(channel)
                if name in TELEMETRY:
                    state.ingest(t, name, deserialize(raw))
                elif name == 'oak.color' and images is not None:
                    data = deserialize(raw)
                    # Cache parameter sets while catching up; avoid decoding historical frames.
                    if catching_up:
                        if data.startswith(b'\xff\xd8'):
                            pending_rgb = (t, data)
                        for match in START_CODE.finditer(data):
                            index = match.end()
                            kind = data[index] >> 1 & 63 if index < len(data) else -1
                            if 16 <= kind <= 21:
                                pending_rgb = (t, data)
                            if kind in (32, 33, 34):
                                end = START_CODE.search(data, index)
                                images.parameters[kind] = data[match.start() : end.start() if end else len(data)]
                        continue
                    try:
                        image = images.image(data)
                    except Exception as error:
                        with state.lock:
                            state.rgb_error = str(error)[:160]
                        continue
                    if image is not None:
                        with state.lock:
                            state.rgb, state.rgb_stamp = image, t
                            state.rgb_size = images.source_size
                            state.rgb_seq += 1
                            state.rgb_error = None
    except Exception as error:
        with state.lock:
            state.error = f'{type(error).__name__}: {error}'[:256]


def handler(state, budget, projection=None):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(2.0)

        def log_message(self, *args):
            pass

        def do_GET(self):
            url = urlsplit(self.path)
            path = url.path
            extra = {}
            if path == '/project':
                if projection is None:
                    self.send_error(404, 'No camera calibration configured')
                    return
                try:
                    query = parse_qs(url.query, strict_parsing=True, max_num_fields=4)
                    session, seq = query['session'][0], int(query['seq'][0])
                    x, y = float(query['x'][0]), float(query['y'][0])
                    current = state.current if isinstance(state, DirectorySource) else state
                    with current.lock:
                        if session != current.session_id or seq != current.rgb_seq:
                            self.send_error(409, 'Displayed frame has changed; select the current snapshot')
                            return
                        size, stamp = current.rgb_size, current.rgb_stamp
                    target = projection.project(x, y, size)
                    body = json.dumps(
                        dict(target_robot_m=target, session_id=session, rgb_seq=seq, log_time_s=stamp)
                    ).encode()
                except (KeyError, ValueError, TypeError) as error:
                    self.send_error(400, str(error))
                    return
                content_type = 'application/json'
            elif path == '/state':
                snapshot = state.snapshot()
                snapshot['projection_enabled'] = projection is not None
                body = json.dumps(snapshot, separators=(',', ':'), allow_nan=False).encode()
                content_type = 'application/json'
            elif path == '/rgb':
                body, extra['ETag'] = state.get_rgb()
                if body is None:
                    self.send_error(404, 'No RGB snapshot yet')
                    return
                if self.headers.get('If-None-Match') == extra['ETag']:
                    self.send_response(304)
                    self.end_headers()
                    return
                content_type = 'image/jpeg'
            elif path == '/':
                body = Path(__file__).with_suffix('.html').read_bytes()
                content_type = 'text/html; charset=utf-8'
            else:
                self.send_error(404)
                return
            reserve = min(budget.rate, budget.capacity / 4) if path == '/rgb' else 0
            if not budget.take(len(body), reserve=reserve):
                self.send_response(429)
                self.send_header('Retry-After', '2')
                self.send_header('Content-Length', '0')
                self.end_headers()
                return
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            for key, value in extra.items():
                self.send_header(key, value)
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'log', type=Path, help='Existing log file (fixed) or directory (automatically follow newest recording)'
    )
    parser.add_argument('--pattern', default='*.log', help='Filename glob for directory mode; default: *.log')
    parser.add_argument(
        '--calibration', type=Path, help='Optional RGB intrinsics and camera mounting JSON for ground preview'
    )
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument(
        '--kib-per-second', type=float, default=8.0, help='Response body budget; headers/SSH overhead additional'
    )
    parser.add_argument(
        '--rgb-interval', type=float, default=5.0, help='Minimum seconds between snapshot attempts; 0 disables decoding'
    )
    args = parser.parse_args()
    if not (args.log.is_file() or args.log.is_dir()) or not 1 <= args.port <= 65535:
        parser.error('Need an existing log file or directory and valid port')
    if '/' in args.pattern or '\\' in args.pattern or not args.pattern:
        parser.error('pattern must be a filename glob without directories')
    if not math.isfinite(args.rgb_interval) or args.rgb_interval < 0:
        parser.error('rgb-interval must be nonnegative')
    if not math.isfinite(args.kib_per_second) or args.kib_per_second <= 0:
        parser.error('kib-per-second must be positive')
    projection = None
    if args.calibration is not None:
        try:
            projection = GroundProjection(json.loads(args.calibration.read_text()))
        except (OSError, ValueError, KeyError, TypeError) as error:
            parser.error(f'Invalid camera calibration: {error}')
    state = DirectorySource(args.log) if args.log.is_dir() else State(args.log)
    stop = threading.Event()
    budget = ByteBudget(args.kib_per_second * 1024)
    with ThreadingHTTPServer(('127.0.0.1', args.port), handler(state, budget, projection)) as server:
        if isinstance(state, DirectorySource):
            target, arguments = follow_directory, (state, stop, args.rgb_interval, args.pattern)
        else:
            target, arguments = follow, (state, stop, args.rgb_interval)
        threading.Thread(target=target, args=arguments, daemon=True).start()
        print(f'Read-only monitor: http://127.0.0.1:{args.port} ; Ctrl+C stops only this monitor', flush=True)
        try:
            server.serve_forever(poll_interval=0.2)
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()


if __name__ == '__main__':
    main()
