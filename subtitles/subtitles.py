"""
Subtitles Generator for OSGAR Logs

Parses OSGAR logs for target platform 'Pat' to extract 'platform.manual' stream,
which indicates whether the platform is in AUTONOMOUS (platform.manual is False)
or MANUAL (platform.manual is True) mode. Generates a SubRip (SRT) subtitles file
for YouTube with correct timestamps and an optional video time offset.
"""

import argparse
import datetime
import os
import sys

from osgar.logger import LogReader, LogReaderEx, LogIndexedReader


def get_log_duration(logfile: str) -> float:
    """
    Get the overall duration of the log file in seconds.
    Uses LogIndexedReader for speed and falls back to LogReader if needed.
    """
    try:
        with LogIndexedReader(logfile) as reader:
            if reader.index:
                return reader.index[-1][1].total_seconds()
    except Exception:
        pass

    with LogReader(logfile) as log:
        last_dt = None
        for dt, _, _ in log:
            last_dt = dt
        if last_dt is not None:
            return last_dt.total_seconds()

    return 0.0


def format_srt_timestamp(seconds: float) -> str:
    """
    Format seconds (float) to SRT timestamp format: HH:MM:SS,mmm
    Handles clamping of negative values to 0.0.
    """
    if seconds < 0:
        seconds = 0.0
    total_ms = int(round(seconds * 1000))
    ms = total_ms % 1000
    total_secs = total_ms // 1000
    secs = total_secs % 60
    total_mins = total_secs // 60
    mins = total_mins % 60
    hours = total_mins // 60
    return f'{hours:02d}:{mins:02d}:{secs:02d},{ms:03d}'


def generate_segments(events, log_end_time):
    """
    events: list of (log_time, mode) where log_time is float and mode is str
    log_end_time: float
    Returns a list of (start_log_time, end_log_time, mode)
    """
    if not events:
        return []

    # Deduplicate consecutive identical modes
    dedup = []
    for log_time, mode in events:
        if not dedup or dedup[-1][1] != mode:
            dedup.append((log_time, mode))

    segments = []
    for i in range(len(dedup)):
        start_time = dedup[i][0]
        end_time = dedup[i + 1][0] if i + 1 < len(dedup) else log_end_time

        if start_time < end_time:
            segments.append((start_time, end_time, dedup[i][1]))

    return segments


def apply_offset(segments, offset_sec):
    """
    segments: list of (start_log_time, end_log_time, mode)
    offset_sec: float
    Returns a list of (start_video_time, end_video_time, mode)
    """
    video_segments = []
    for start, end, mode in segments:
        v_start = start + offset_sec
        v_end = end + offset_sec

        # Skip segments that end before video starts
        if v_end <= 0:
            continue

        # Clamp segment start to 0.0 if it starts before video starts
        if v_start < 0:
            v_start = 0.0

        video_segments.append((v_start, v_end, mode))
    return video_segments


def write_srt(video_segments, output_file):
    """
    Writes a list of video segments to an SRT file.
    """
    with open(output_file, 'w', encoding='utf-8') as f:
        for idx, (start, end, mode) in enumerate(video_segments, 1):
            f.write(f'{idx}\n')
            f.write(f'{format_srt_timestamp(start)} --> {format_srt_timestamp(end)}\n')
            f.write(f'Mode: {mode}\n\n')


def process_log_to_subtitles(logfile: str, output_srt: str, offset_sec: float = 0.0):
    """
    Reads log file, extracts platform.manual stream, processes transitions and
    writes output SRT file.
    """
    if not os.path.exists(logfile):
        raise FileNotFoundError(f"Log file '{logfile}' does not exist.")

    manual_stream = 'platform.manual'
    events = []

    try:
        with LogReaderEx(logfile, names=[manual_stream]) as log:
            for dt, stream, data in log:
                if stream == manual_stream:
                    # data is boolean
                    mode = 'MANUAL' if data else 'AUTONOMOUS'
                    events.append((dt.total_seconds(), mode))
    except ValueError:
        # If platform.manual stream does not exist in the log
        pass

    if not events:
        log_duration = get_log_duration(logfile)
        print(f"Warning: Stream '{manual_stream}' not found in log file '{logfile}'.")
        print(f'Total duration of log is {log_duration:.3f}s, but no mode events were found.')
        print("No 'platform.manual' events found. No subtitle file will be written.")
        return

    log_end_time = get_log_duration(logfile)

    # If the last event timestamp is greater than log_end_time, adjust log_end_time
    if events and events[-1][0] > log_end_time:
        log_end_time = events[-1][0]

    segments = generate_segments(events, log_end_time)
    video_segments = apply_offset(segments, offset_sec)

    # Write subtitles to SRT file
    write_srt(video_segments, output_srt)
    print(f"Successfully wrote {len(video_segments)} subtitle segments to '{output_srt}'.")


def main():
    parser = argparse.ArgumentParser(
        description='Generate SRT subtitles for YouTube from OSGAR platform.manual log stream.'
    )
    parser.add_argument('logfile', help='Path to the input OSGAR log file')
    parser.add_argument('output_srt', help='Path to the output SRT subtitles file')
    parser.add_argument(
        '--offset',
        type=float,
        default=0.0,
        help='Video offset in seconds (added to log timestamps, default: 0.0)'
    )
    args = parser.parse_args()

    process_log_to_subtitles(args.logfile, args.output_srt, args.offset)


if __name__ == '__main__':
    main()
