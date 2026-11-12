# Subtitles Generator for OSGAR Logs (Platform Pat)

This tool generates a SubRip (SRT) subtitles file for YouTube based on remote mode transitions of robot platform "Pat".
The platform switches between **AUTONOMOUS** and **MANUAL** modes, which are recorded as a Boolean in the `platform.manual` stream.

## Features

- Extracts `platform.manual` events directly from OSGAR logs.
- Identifies mode transitions and builds continuous subtitle blocks:
  - `platform.manual = False` $\rightarrow$ **AUTONOMOUS**
  - `platform.manual = True` $\rightarrow$ **MANUAL**
- Deduplicates successive status entries automatically.
- Adjusts the output subtitle timeline with a custom video time offset in seconds.
- Clamps and handles negative times safely for compatibility with YouTube SRT specifications.

## Installation / Requirements

Ensure you are using the project's virtual environment or have `osgar` installed:

```bash
uv sync
```

## Usage

Run the generator script using Python:

```bash
python -m subtitles.subtitles <logfile> <output_srt> [--offset <offset_sec>]
```

### Arguments

- `logfile`: Path to the input OSGAR log file (e.g., `data/pat-platform.log`).
- `output_srt`: Target path to write the generated SRT subtitles file.
- `--offset`: Video offset in seconds (added to log timestamps, can be a positive or negative float, default: `0.0`).

### Example

Generate subtitles with standard timestamps:

```bash
python -m subtitles.subtitles data/pat-platform.log subtitles.srt
```

Generate subtitles with a video that is 12.5 seconds behind the log (offset of +12.5s):

```bash
python -m subtitles.subtitles data/pat-platform.log subtitles.srt --offset 12.5
```

## Running Tests

All unit tests are integrated with the global test suite and run automatically. You can also run them independently:

```bash
python -m unittest subtitles/test_subtitles.py
```
