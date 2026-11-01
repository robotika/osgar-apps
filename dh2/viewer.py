"""Interactive DH2 log viewer: raw color, separate depth and recorded telemetry."""

import argparse
import os
import re
import textwrap
import time
from bisect import bisect_right
from fractions import Fraction
from pathlib import Path

import av
import cv2
import numpy as np
from osgar.lib.serialize import deserialize
from osgar.logger import LogIndexedReader, lookup_stream_names

CHANNELS = ('oak.color', 'oak.depth', 'app.status', 'app.desired_steering',
            'app.defender', 'platform.bumpers_front', 'platform.bumpers_rear', 'platform.emergency_stop')
START_CODE = re.compile(b'\x00\x00\x00?\x01')


def hevc_units(data):
    """Return Annex B NAL units, including start codes."""
    starts = [m.start() for m in START_CODE.finditer(data)]
    return [(data[START_CODE.match(data, a).end()] >> 1 & 63, data[a:b])
            for a, b in zip(starts, starts[1:] + [len(data)])
            if START_CODE.match(data, a).end() < b]


class Recording:
    """Index packet locations, not decoded images; seek from HEVC keyframes."""

    def __init__(self, filename):
        self.filename = Path(filename)
        names = lookup_stream_names(str(filename))
        self.channels = {names.index(n) + 1: n for n in CHANNELS if n in names}
        self.log = LogIndexedReader(str(filename))
        self.rows = {n: [] for n in self.channels.values()}
        self.keys = []
        self.parameters = {}
        self.cache = {}

    def __enter__(self):
        self.log.__enter__()
        try:
            for index in range(len(self.log)):
                stamp, channel, raw = self.log[index]
                name = self.channels.get(channel)
                if name is None:
                    continue
                sec = stamp.total_seconds()
                self.rows[name].append((sec, index))
                if name == 'oak.color':
                    units = hevc_units(deserialize(raw))
                    for kind, unit in units:
                        if kind in (32, 33, 34) and kind not in self.parameters:
                            self.parameters[kind] = unit
                    if any(16 <= kind <= 21 for kind, _ in units):
                        self.keys.append((sec, len(self.rows[name]) - 1))
            self.times = {n: [t for t, _ in rows] for n, rows in self.rows.items()}
            if not self.keys:
                raise ValueError(f'{self.filename}: no HEVC color keyframes (possibly a camera-free STOP run)')
            self.duration = self.rows['oak.color'][-1][0]
            return self
        except Exception:
            self.log.__exit__(None, None, None)
            raise

    def __exit__(self, *args):
        self.log.__exit__(*args)

    def latest(self, name, sec):
        """Latest recorded value at or before sec; never borrow a future sample."""
        i = bisect_right(self.times.get(name, []), sec) - 1
        if i < 0:
            return None, None
        stamp, index = self.rows[name][i]
        if self.cache.get(name, (None,))[0] != index:
            self.cache[name] = (index, deserialize(self.log[index][2]))
        return stamp, self.cache[name][1]

    def frames(self, start=0):
        key = max(0, bisect_right(self.keys, (start, float('inf'))) - 1)
        decoder = av.CodecContext.create('hevc', 'r')
        decoder.thread_count = 2
        decoder.decode(av.Packet(b''.join(self.parameters[k] for k in sorted(self.parameters))))
        time_base = Fraction(1, 1_000_000)
        for sec, index in self.rows['oak.color'][self.keys[key][1]:]:
            packet = av.Packet(deserialize(self.log[index][2]))
            packet.pts = round(sec * 1_000_000)
            packet.time_base = time_base
            for frame in decoder.decode(packet):
                timestamp = float(frame.pts * (frame.time_base or time_base))
                if timestamp >= start:
                    yield timestamp, frame.to_ndarray(format='bgr24')
        for frame in decoder.decode(None):
            # PyAV may omit the time base on delayed frames returned by flushing.
            timestamp = float(frame.pts * (frame.time_base or time_base))
            if timestamp >= start:
                yield timestamp, frame.to_ndarray(format='bgr24')


def depth_image(depth, near, far):
    valid = np.isfinite(depth) & (depth > 0)
    values = np.where(valid, depth, 0).astype(float) / 1000
    scale = (255 * (1 - np.clip((values - near) / (far - near), 0, 1))).astype(np.uint8)
    color = cv2.applyColorMap(scale, cv2.COLORMAP_TURBO)
    color[~valid] = 0
    return color


def fit(image, width, height):
    """Letterbox without distorting the camera's aspect ratio."""
    scale = min(width / image.shape[1], height / image.shape[0])
    resized = cv2.resize(image, (round(image.shape[1] * scale), round(image.shape[0] * scale)))
    result = np.zeros((height, width, 3), dtype=np.uint8)
    y, x = (height - resized.shape[0]) // 2, (width - resized.shape[1]) // 2
    result[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    return result


def layout(recording, frame, *, near=0.3, far=5, speed=1, paused=False):
    sec, color = frame
    image = np.full((720, 1280, 3), (28, 21, 15), dtype=np.uint8)

    def text(x, y, value, scale=0.55, color=(235, 235, 235)):
        cv2.putText(image, str(value), (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)

    text(20, 33, 'MATTY / DH2', 0.75)
    text(250, 33, recording.filename.name, 0.6)
    text(1000, 33, f'{speed:g}x  {"PAUSED" if paused else "PLAY"}')
    image[70:610, :960] = fit(color, 960, 540)
    text(975, 92, 'STEREO DEPTH', 0.6)
    stamp, depth = recording.latest('oak.depth', sec)
    if depth is not None:
        image[110:300, 968:1272] = fit(depth_image(depth, near, far), 304, 190)
        text(975, 329, f'Age: {sec - stamp:.3f} s')
    else:
        text(975, 210, 'No depth sample')
    text(975, 355, f'Near {near:g} m -> far {far:g} m', 0.48)
    text(975, 380, 'Black: no measurement', 0.48)
    text(975, 405, 'Left camera; not RGB aligned', 0.43)
    text(975, 440, f'Log +{sec:.2f} s / {recording.duration:.2f} s', 0.47)
    _, steering = recording.latest('app.desired_steering', sec)
    if steering is not None:
        text(975, 473, f'Command: {steering[0] / 1000:.2f} m/s', 0.48)
        text(975, 498, f'Steering: {steering[1] / 100:.1f} deg', 0.48)
    for y, label, channel in [(530, 'Front bumper', 'platform.bumpers_front'),
                              (554, 'Rear bumper', 'platform.bumpers_rear'),
                              (578, 'STOP', 'platform.emergency_stop')]:
        _, value = recording.latest(channel, sec)
        text(975, y, f'{label}: {"?" if value is None else bool(value)}', 0.48)
    report_time, _ = recording.latest('app.defender', sec)
    if report_time is not None:
        text(975, 604, f'Last report: +{report_time:.2f}s', 0.45, (155, 240, 120))
    _, status = recording.latest('app.status', sec)
    for y, line in zip((638, 663), textwrap.wrap(str(status or 'No recorded app.status'), 100)):
        text(20, y, line, 0.58)
    text(20, 703, 'Space: pause  .: step  A/D: -/+5s  +/-: speed  N/P: next/previous log  Q: quit', 0.5)
    return image


def play(recording, args, index, total):
    window = 'DH2 recordings'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1280, 780)
    pending_seek = None
    updating = False

    def slider(value):
        nonlocal pending_seek
        if not updating:
            pending_seek = value / 10

    cv2.createTrackbar('Log time (0.1s)', window, 0, max(1, int(recording.duration * 10)), slider)
    pending_seek = None
    frames = recording.frames(args.start)
    current = next(frames, None)
    if current is None:
        print(f'{recording.filename}: no color at/after {args.start}s; skipping')
        return 1
    following = None
    paused, speed = args.pause, args.speed
    shown_at = time.monotonic()
    dirty = True
    while True:
        if pending_seek is not None:
            frames = recording.frames(min(pending_seek, recording.duration))
            current = next(frames, current)
            following = None
            pending_seek = None
            shown_at = time.monotonic()
            dirty = True
        if dirty:
            cv2.imshow(window, layout(recording, current, near=args.near, far=args.far, speed=speed, paused=paused))
            updating = True
            cv2.setTrackbarPos('Log time (0.1s)', window, int(current[0] * 10))
            updating = False
            dirty = False
        key = cv2.waitKeyEx(10)
        if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1 or key in (ord('q'), 27):
            return 0
        if key == ord('n'):
            return 1
        if key == ord('p'):
            return -1
        if key == ord(' '):
            paused = not paused
            shown_at = time.monotonic()
            dirty = True
        if key in (ord('+'), ord('='), ord('-')):
            speed = max(0.125, min(16, speed * (0.5 if key == ord('-') else 2)))
            shown_at = time.monotonic()
            dirty = True
        if key in (ord('a'), ord('d'), 65361, 65363, 2424832, 2555904):
            pending_seek = max(0, current[0] + (-5 if key in (ord('a'), 65361, 2424832) else 5))
            continue
        if not paused or key == ord('.'):
            if following is None:
                following = next(frames, None)
            if following is None:
                if index + 1 < total:
                    return 1
                paused = True
                dirty = True
            elif key == ord('.') or time.monotonic() >= shown_at + (following[0] - current[0]) / speed:
                shown_at = (time.monotonic() if key == ord('.') else
                            shown_at + (following[0] - current[0]) / speed)
                current, following = following, None
                paused = paused or key == ord('.')
                dirty = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', type=Path, nargs='+', help='Recordings in playback order (shell wildcards supported)')
    parser.add_argument('--start', type=float, default=0, help='Initial log offset in seconds, applied to each log')
    parser.add_argument('--speed', type=float, default=1, help='Initial playback speed (default: 1)')
    parser.add_argument('--pause', action='store_true', help='Start paused')
    parser.add_argument('--near', type=float, default=0.3, help='Near end of depth display scale, meters')
    parser.add_argument('--far', type=float, default=5, help='Far end of depth display scale, meters')
    parser.add_argument('--snapshot', type=Path, help='Save one PNG/JPEG without opening a GUI (one input log)')
    args = parser.parse_args()
    if not (0 <= args.start and 0 < args.speed and 0 < args.near < args.far):
        parser.error('require start >= 0, speed > 0 and 0 < near < far')
    if args.snapshot and len(args.logs) != 1:
        parser.error('--snapshot requires exactly one log')
    if not args.snapshot and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        parser.error('no graphical display; use --snapshot for headless rendering')
    index = 0
    try:
        while 0 <= index < len(args.logs):
            print(f'Indexing {args.logs[index]} ...', flush=True)
            try:
                with Recording(args.logs[index]) as recording:
                    if args.snapshot:
                        frame = next(recording.frames(args.start), None)
                        if frame is None:
                            parser.error('no color frame at/after requested offset')
                        image = layout(recording, frame, near=args.near, far=args.far, paused=True)
                        if not cv2.imwrite(str(args.snapshot), image):
                            raise OSError(f'Could not write {args.snapshot}')
                        return
                    direction = play(recording, args, index, len(args.logs))
                    cv2.destroyAllWindows()
                    if direction == 0:
                        break
                    index = max(0, index - 1) if direction < 0 else index + 1
            except ValueError as error:
                print(error)
                if args.snapshot:
                    parser.exit(1)
                index += 1
    finally:
        if not args.snapshot:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
