# `config/`

This folder stores Basalt JSON assets used by the wrapper.

Typical contents:

- Basalt camera + IMU calibration JSON files
- Basalt VIO config JSON files
- local sample files used for smoke tests or dataset presets

Notes:

- The calibration file must be a Basalt-compatible JSON that deserializes into `basalt::Calibration<double>`. Do not invent a custom format.
- Example files from Basalt can be useful smoke tests to prove the node starts, but they are not a final calibration for your own sensor rig.
- A config/calibration mismatch is one of the fastest ways to trigger startup crashes or invalid odometry.
- Keep the paths stable and pass them in through the launch arguments `calib_path` and `config_path`.
