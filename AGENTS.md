# Working in osgar-apps

## Scope and simplicity

- Keep application-specific mission behavior and configurations here. OSGAR core
  is shared by multiple teams and should stay relatively simple and generic.
- Prefer a small, reviewable MVP. Reuse OSGAR facilities rather than duplicating
  framework responsibilities in application nodes.
- OSGAR controls runtime through `osgar.record --duration`; do not add a second
  application runtime limit for the same purpose.
- OSGAR prepares configured devices at startup. Do not introduce ROS-style
  late-joining sensor support or general sensor-availability watchdogs by default.
  Waiting for initial messages and handling lost targets or invalid depth are
  separate application concerns.

## Configuration and robot identity

- Place application configurations in a `config/` subfolder, e.g.
  `dh2/config/dh2.json`.
- Prefer one shared configuration over copies for individual robots. Follow
  `dtc-systems/dtc.py` for conditional settings: read `OSGAR_LOGS_PREFIX` from
  `config['env']` and look up prefix-specific keys with a common fallback.
- The correct environment variable is `OSGAR_LOGS_PREFIX`. Declare it as null in
  the configuration's `robot.env` so OSGAR supplies the launch environment value.
  Robot prefixes are added automatically to log filenames; do not encode robot
  names in otherwise identical config filenames.
- Check the working directory when configuring relative model paths.

## Environment and validation

- Use the repository's Python 3.10 uv environment and lockfile; see `README.md`.
  The current NumPy pin limits interpreter compatibility. Do not silently upgrade
  the interpreter or dependency stack as part of an unrelated application edit.
- Follow `DEVELOPMENT.md` for work based on robot logs: verify baseline replay,
  analyze, implement, inspect normal replay differences, then force replay to
  evaluate the new outputs. Forced replay does not validate the new physical
  trajectory.
- Run focused tests and lint for changed code. Report hardware validation
  separately from synthetic tests and source/configuration checks.
- `/data/matty` contains many unsorted historical recordings on the operator's
  workstation. Look there for relevant logs before assuming new recordings are
  needed; a searchable index is future work.

## Documentation and commits

- Keep project handoff documents in `doc/`. DH2 context and branch review are in
  `doc/dh2-handoff.md`; current implementation details belong in `dh2/README.md`.
- When asked to commit, use the established agent identity for author and
  committer: `Aya Rain <aya.rain@robotika.cz>`. Set it per command rather than
  changing the operator's Git identity.
- Stage only files within the requested scope. Leave unrelated local files such
  as `.idea/` untouched. Do not infer a request to commit from a request to edit.
