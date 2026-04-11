# basalt_wrapper

`basalt_wrapper` is a generic ROS 2 wrapper around Basalt VIO.

The package does only this:

- subscribe to ROS image and IMU topics
- run Basalt
- publish generic ROS odometry and debugging outputs

## Scope

This package is intentionally limited to generic ROS 2 integration around
Basalt. It does not contain:

- PX4-specific bridges
- loop closure or SLAM
- dataset conversion tooling
- simulator-specific glue code

That separation keeps the wrapper reusable across bags, robots, and downstream
consumers.

## Package Layout

- `src/`: wrapper node implementation
- `launch/`: launch entrypoints
- `params/`: reusable dataset parameter presets
- `config/`: Basalt JSON configs and calibration references
- `rviz/`: RViz display configuration

Published topics:

- `/basalt/pose` as `geometry_msgs/msg/PoseStamped`
- `/basalt/path` as `nav_msgs/msg/Path`
- `/basalt/odometry` as `nav_msgs/msg/Odometry`
- `/basalt/pose_cloud` as `sensor_msgs/msg/PointCloud`
- `/basalt/tracking_image` as `sensor_msgs/msg/Image`
- `/basalt/tracking_overlay` as `sensor_msgs/msg/Image`
- `/basalt/tracked_points` as `sensor_msgs/msg/PointCloud`

Debug service:

- `/basalt/get_debug_snapshot` as `std_srvs/srv/Trigger`

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Git
- a C++17 toolchain
- Python 3 with `pip`

## Tested Basalt Revision

This wrapper is currently pinned and tested against:

- Basalt repository:
  - `https://github.com/VladyslavUsenko/basalt.git`
- branch:
  - `master`
- commit:
  - `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`

The repository ships a bootstrap script that clones and builds that exact
Basalt revision into:

```bash
third_party/basalt
```

You can still override the Basalt location with:

```bash
--cmake-args -DBASALT_ROOT=/path/to/basalt
```

## Bootstrap Basalt

From the package root:

```bash
chmod +x scripts/setup_basalt.sh
./scripts/setup_basalt.sh
```

That clones the pinned Basalt revision into `third_party/basalt` and builds it
under `third_party/basalt/build/relwithdebinfo`.

If your system `cmake` is older than `3.24`, the script installs a newer one
with `python3 -m pip install --user cmake`.

## Inputs

Images:

- mono:
  - `left_image_topic`
- stereo:
  - `left_image_topic`
  - `right_image_topic`

IMU:

- `imu_topic` as `sensor_msgs/msg/Imu`

## Main Parameters

- `left_image_topic`
- `right_image_topic`
- `imu_topic`
- `calib_path`
- `config_path`
- `path_frame_id`
- `body_frame_id`
- `use_imu`
- `use_header_timestamps`
- `max_path_length`

## Build

Exact build command using the bundled Basalt checkout:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
cd src/basalt_wrapper
chmod +x scripts/setup_basalt.sh
./scripts/setup_basalt.sh
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select basalt_wrapper --cmake-clean-cache
source install/setup.bash
```

Exact build command using an external Basalt checkout:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select basalt_wrapper --cmake-clean-cache \
  --cmake-args -DBASALT_ROOT=/path/to/basalt
source install/setup.bash
```

## Launch

Exact run command for EuRoC-style stereo + IMU input:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch basalt_wrapper basalt_node.launch.py \
  left_image_topic:=/cam0/image_raw \
  right_image_topic:=/cam1/image_raw \
  imu_topic:=/imu0 \
  calib_path:=/path/to/basalt/data/euroc_eucm_calib.json \
  config_path:=/path/to/basalt/data/euroc_config.json \
  use_rviz:=true
```

Exact run command using the packaged EuRoC preset:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch basalt_wrapper dataset_vio.launch.py \
  config_file:=/path/to/your/ws/src/basalt_wrapper/params/euroc_vio.params.yaml \
  use_rviz:=true
```

Then replay your bag in a second terminal:

```bash
ros2 bag play /path/to/bag_folder --storage sqlite3 --clock
```

## Expected Topics

When the node is running, these are the main public topics and services:

- `/basalt/pose`
- `/basalt/path`
- `/basalt/odometry`
- `/basalt/pose_cloud`
- `/basalt/tracking_image`
- `/basalt/tracking_overlay`
- `/basalt/tracked_points`
- `/basalt/get_debug_snapshot`

## Included Presets

- `params/euroc_vio.params.yaml`
- `params/tumvi_vio.params.yaml`
- `params/openloris_mono_vo.params.yaml`
- `rviz/basalt_wrapper.rviz`

## RViz

Recommended displays:

- `Path` -> `/basalt/path`
- `Pose` -> `/basalt/pose`
- `Odometry` -> `/basalt/odometry`
- `PointCloud` -> `/basalt/pose_cloud`
- `Image` -> `/basalt/tracking_image`
- `Image` -> `/basalt/tracking_overlay`
- `PointCloud` -> `/basalt/tracked_points`

Recommended fixed frame:

- `basalt_world`

You can launch the packaged RViz view by passing `use_rviz:=true` to either
launch file.

## Supported Inputs

- Mono image + IMU
- Stereo image + IMU
- `sensor_msgs/msg/Imu`
- image encodings:
  - `mono8`
  - `rgb8`
  - `bgr8`

## Dataset Presets

Included presets are intended as starting points, not guarantees:

- `params/euroc_vio.params.yaml`
- `params/tumvi_vio.params.yaml`
- `params/openloris_mono_vo.params.yaml`

You are still responsible for supplying a calibration JSON and config JSON that
match the actual dataset or sensor rig.

## Notes

- `/basalt/tracked_points` is a 2D image-space point cloud:
  - `x`, `y` are image pixel coordinates
  - channel `feature_id` is the Basalt keypoint id
  - channel `camera_index` is the camera index
- `/basalt/pose_cloud` is a persistent breadcrumb cloud in the path frame and is useful for visually spotting drift.
- This package no longer publishes PX4 topics and no longer depends on PX4 message types.

## Known Limitations

- The wrapper depends on a local Basalt source/build tree instead of a system package.
- Calibration and estimator config quality dominate output quality; the wrapper does not solve bad sensor models.
- Basalt VIO is not loop-closing SLAM, so long trajectories can drift.
- The package does not currently ship automated integration tests against public datasets.
