# Contributing

## Development Environment

- ROS 2 Humble
- Ubuntu 22.04
- Python 3 with `pip`
- Basalt built locally and discoverable through `BASALT_ROOT` or `~/basalt`

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select basalt_wrapper --cmake-clean-cache
source install/setup.bash
```

If Basalt is installed somewhere other than `~/basalt`, point CMake at it:

```bash
colcon build --packages-select basalt_wrapper --cmake-clean-cache \
  --cmake-args -DBASALT_ROOT=/path/to/basalt
```

## Contribution Scope

Good contributions are:

- wrapper input/output improvements
- launch and parameter preset improvements
- better debug and visualization topics
- documentation, examples, and dataset presets
- packaging and build-system improvements

Changes that should stay in separate packages:

- PX4-specific bridges
- dataset converters
- simulator-specific integrations

## Before Opening a PR

- make sure the package builds in a clean ROS 2 workspace
- update `README.md` if user-facing behavior changes
- keep launch files and parameter presets in sync
- avoid introducing machine-specific absolute paths except clearly marked placeholders
