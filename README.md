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
- simulator-specific glue code

That separation keeps the wrapper reusable across bags, robots, and downstream
consumers.

## Package Layout

- `src/`: wrapper node implementation
- `launch/`: launch entrypoints
- `params/`: reusable dataset parameter presets
- `config/`: notes about expected Basalt calibration/config assets
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

The default build path now vendors Basalt automatically on the first `colcon
build`, so you do not need to run the bootstrap script manually unless you want
to prefetch or debug the dependency in isolation.

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

Exact build command using the default vendored Basalt checkout:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select basalt_wrapper --cmake-clean-cache
source install/setup.bash
```

On the first build, `basalt_wrapper` will clone and build the pinned Basalt
revision into `src/basalt_wrapper/third_party/basalt`. Later builds reuse that
vendored checkout.

Exact build command using an external Basalt checkout:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select basalt_wrapper --cmake-clean-cache \
  --cmake-args -DBASALT_ROOT=/path/to/basalt
source install/setup.bash
```

## Current Accuracy Status

Current measured results on EuRoC `V1_02_medium`:

- native `basalt_vio` on the original EuRoC folder:
  - `APE RMSE = 0.045336 m`
- `basalt_wrapper` on the offline-converted rosbag2:
  - best observed run so far:
    - `APE RMSE = 2.219359 m`
  - additional recent runs:
    - `APE RMSE = 3.586488 m`
    - `APE RMSE = 6.432690 m`

Interpretation:

- native Basalt is working correctly on the dataset
- the wrapper is much better than the earlier tens-to-thousands-of-meters failure
  mode
- the wrapper is still materially worse than native Basalt and should not yet be
  treated as numerically equivalent for evaluation

These numbers are here to document the current state of the integration, not as
an acceptance target.

## Launch

The wrapper exposes a single generic launch entrypoint. You are expected to
choose the topics and Basalt calibration/config files explicitly for your
dataset or sensor rig.

For EuRoC MAV, use the same Basalt calibration and config files that native
`basalt_vio` uses:

- `data/euroc_eucm_calib.json`
- `data/euroc_config.json`

The difference is only the input transport:

- native Basalt reads directly from a dataset folder
- `basalt_wrapper` reads the same data through ROS 2 topics from a played bag

Example run command for EuRoC-style stereo + IMU input:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch basalt_wrapper basalt_node.launch.py \
  left_image_topic:=/cam0/image_raw \
  right_image_topic:=/cam1/image_raw \
  imu_topic:=/imu0 \
  calib_path:=$HOME/ros2_ws/src/basalt_wrapper/third_party/basalt/data/euroc_eucm_calib.json \
  config_path:=$HOME/ros2_ws/src/basalt_wrapper/third_party/basalt/data/euroc_config.json \
  use_rviz:=true
```

Then replay your bag in a second terminal:

```bash
ros2 bag play /path/to/euroc_rosbag2_folder --storage sqlite3 --clock
```

## EuRoC Conversion

For fidelity-critical evaluation, do not use the live replay-and-record helper as
your primary conversion path. The preferred path is the offline converter below,
because it writes the original EuRoC timestamps directly into both:

- `msg.header.stamp`
- the rosbag2 message write timestamp

and it writes all topics in one globally timestamp-sorted stream.

### Preferred: Offline EuRoC to rosbag2

This package ships an offline converter that writes:

- `/cam0/image_raw`
- `/cam0/camera_info`
- `/cam1/image_raw`
- `/cam1/camera_info`
- `/imu0`

from an original EuRoC `mav0` folder or sequence root.

Example:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run basalt_wrapper euroc_to_rosbag2 \
  --ros-args \
  -p dataset_path:=/home/caio/vicon_room1/vicon_room1/V1_02_medium \
  -p output_uri:=/home/caio/V1_02_medium_offline_ros2 \
  -p storage_id:=sqlite3
```

Then inspect the generated bag:

```bash
ros2 bag info /home/caio/V1_02_medium_offline_ros2
```

Replay it into the wrapper with simulated time:

```bash
ros2 bag play /home/caio/V1_02_medium_offline_ros2 --clock
```

## End-to-End EuRoC Workflow

This is the current recommended EuRoC workflow for `V1_02_medium`.

### 1. Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select basalt_wrapper --cmake-clean-cache
source install/setup.bash
```

### 2. Convert the original EuRoC dataset offline

```bash
rm -rf /home/caio/V1_02_medium_offline_ros2

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run basalt_wrapper euroc_to_rosbag2 \
  --ros-args \
  -p dataset_path:=/home/caio/vicon_room1/vicon_room1/V1_02_medium \
  -p output_uri:=/home/caio/V1_02_medium_offline_ros2 \
  -p storage_id:=sqlite3
```

### 3. Verify the bag

```bash
ros2 bag info /home/caio/V1_02_medium_offline_ros2
```

Expected topics:

- `/cam0/image_raw`
- `/cam0/camera_info`
- `/cam1/image_raw`
- `/cam1/camera_info`
- `/imu0`

### 4. Run the wrapper on the offline bag

Terminal 1:

```bash
rm -f /home/caio/wrapper_estimated.tum

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch basalt_wrapper basalt_node.launch.py \
  left_image_topic:=/cam0/image_raw \
  right_image_topic:=/cam1/image_raw \
  imu_topic:=/imu0 \
  calib_path:=/home/caio/ros2_ws/src/basalt_wrapper/third_party/basalt/data/euroc_eucm_calib.json \
  config_path:=/home/caio/ros2_ws/src/basalt_wrapper/third_party/basalt/data/euroc_config.json \
  trajectory_output_path:=/home/caio/wrapper_estimated.tum \
  use_rviz:=true \
  use_sim_time:=true
```

Terminal 2:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag play /home/caio/V1_02_medium_offline_ros2 --clock
```

### 5. Compare wrapper output to ground truth

```bash
awk 'NF == 8 {print $0}' /home/caio/wrapper_estimated.tum > /home/caio/wrapper_estimated_clean.tum
source ~/evo-venv/bin/activate
evo_ape tum /home/caio/groundtruth.tum /home/caio/wrapper_estimated_clean.tum --align
```

### 6. Native Basalt baseline

```bash
/home/caio/ros2_ws/src/basalt_wrapper/third_party/basalt/build/relwithdebinfo/basalt_vio \
  --dataset-path /home/caio/vicon_room1/vicon_room1/V1_02_medium \
  --dataset-type euroc \
  --cam-calib /home/caio/ros2_ws/src/basalt_wrapper/third_party/basalt/data/euroc_eucm_calib.json \
  --config-path /home/caio/ros2_ws/src/basalt_wrapper/third_party/basalt/data/euroc_config.json \
  --show-gui true \
  --use-imu true
```

### Legacy: Live Replay Helper

The package also ships a small EuRoC replay helper so you can publish an
original EuRoC MAV folder as ROS 2 topics and record it into a rosbag2.
This is convenient for quick inspection, but it is less faithful than the
offline converter because it introduces ROS scheduling into the conversion path.

Replay the original dataset:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run basalt_wrapper euroc_replay_node \
  --ros-args \
  -p dataset_path:=/home/caio/vicon_room1/vicon_room1/V1_02_medium
```

Then record the topics in another terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag record /cam0/image_raw /cam1/image_raw /imu0 \
  -o /home/caio/V1_02_medium_ros2
```

That bag can then be replayed into `basalt_wrapper` with the standard launch
file.

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

These YAML files are examples and starting points only. They are not separate
launch entrypoints and they do not remove the need to choose matching Basalt
calibration and config files yourself.

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

You can launch the packaged RViz view by passing `use_rviz:=true` to
`basalt_node.launch.py`.

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

- Basalt is vendored from source on the first build; this project does not yet consume a native system package for Basalt.
- Calibration and estimator config quality dominate output quality; the wrapper does not solve bad sensor models.
- Basalt VIO is not loop-closing SLAM, so long trajectories can drift.
- The package does not currently ship automated integration tests against public datasets.
