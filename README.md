# osgar-apps
OSGAR applications

## Development environment

Use Python 3.10, selected by `.python-version`. The current NumPy 1.22.0 pin
supports Python 3.8–3.10; using a newer interpreter requires a separate dependency
update. See the [NumPy release notes](https://numpy.org/doc/1.22/release/1.22.0-notes.html).

From the repository root:

```bash
uv sync --locked
uv run --locked python -m unittest dh2.test_main -v
uv run --locked python -m osgar.replay --help
```

`uv` manages a repository-local `.venv`; activation is optional. The lockfile pins
the OSGAR Git revision as well as Python packages. `pyserial` is included for the
Matty serial driver. Updating OSGAR deliberately requires
`uv lock --upgrade-package osgar` followed by `uv sync --locked`.

The environment can run offline controller tests without a robot. Camera and
serial operation still require access to the actual devices. Avoid starting
`osgar.record` merely to check installation: it opens hardware and can drive
the configured robot.
