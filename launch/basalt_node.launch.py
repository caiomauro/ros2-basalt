# Copyright 2026 Caio Mauro
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Caio Mauro nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    left_image_topic = LaunchConfiguration("left_image_topic")
    right_image_topic = LaunchConfiguration("right_image_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    calib_path = LaunchConfiguration("calib_path")
    config_path = LaunchConfiguration("config_path")
    trajectory_output_path = LaunchConfiguration("trajectory_output_path")
    ingest_log_path = LaunchConfiguration("ingest_log_path")
    diagnostics_log_path = LaunchConfiguration("diagnostics_log_path")
    marg_data_output_path = LaunchConfiguration("marg_data_output_path")
    accepted_input_bag_path = LaunchConfiguration("accepted_input_bag_path")
    accepted_input_bag_cache_mb = LaunchConfiguration("accepted_input_bag_cache_mb")
    global_position_topic = LaunchConfiguration("global_position_topic")
    input_mode = LaunchConfiguration("input_mode")
    bag_uri = LaunchConfiguration("bag_uri")
    bag_storage_id = LaunchConfiguration("bag_storage_id")
    bag_start_time_ns = LaunchConfiguration("bag_start_time_ns")
    bag_preserve_record_order = LaunchConfiguration("bag_preserve_record_order")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    path_frame_id = LaunchConfiguration("path_frame_id")
    body_frame_id = LaunchConfiguration("body_frame_id")
    max_path_length = LaunchConfiguration("max_path_length")
    use_imu = LaunchConfiguration("use_imu")
    use_double = LaunchConfiguration("use_double")
    use_camera_info_calibration = LaunchConfiguration("use_camera_info_calibration")
    camera_model = LaunchConfiguration("camera_model")
    use_header_timestamps = LaunchConfiguration("use_header_timestamps")
    publish_debug_visuals = LaunchConfiguration("publish_debug_visuals")
    publish_replay_inputs = LaunchConfiguration("publish_replay_inputs")
    debug_visual_rate_hz = LaunchConfiguration("debug_visual_rate_hz")
    max_stereo_input_rate_hz = LaunchConfiguration("max_stereo_input_rate_hz")
    use_global_position_factors = LaunchConfiguration("use_global_position_factors")
    imu_startup_calibration_enabled = LaunchConfiguration(
        "imu_startup_calibration_enabled"
    )
    stereo_sync_queue_size = LaunchConfiguration("stereo_sync_queue_size")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    fx = LaunchConfiguration("fx")
    fy = LaunchConfiguration("fy")
    cx = LaunchConfiguration("cx")
    cy = LaunchConfiguration("cy")
    imu_to_cam_translation = LaunchConfiguration("imu_to_cam_translation")
    imu_to_cam_rotation_wxyz = LaunchConfiguration("imu_to_cam_rotation_wxyz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("left_image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("right_image_topic", default_value=""),
            DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
            DeclareLaunchArgument("calib_path", default_value=""),
            DeclareLaunchArgument("config_path", default_value=""),
            DeclareLaunchArgument("trajectory_output_path", default_value=""),
            DeclareLaunchArgument("ingest_log_path", default_value=""),
            DeclareLaunchArgument("diagnostics_log_path", default_value=""),
            DeclareLaunchArgument("marg_data_output_path", default_value=""),
            DeclareLaunchArgument("accepted_input_bag_path", default_value=""),
            DeclareLaunchArgument("accepted_input_bag_cache_mb", default_value="256"),
            DeclareLaunchArgument(
                "global_position_topic", default_value="/geo/vio_position_measurement"
            ),
            DeclareLaunchArgument("input_mode", default_value="ros_topics"),
            DeclareLaunchArgument("bag_uri", default_value=""),
            DeclareLaunchArgument("bag_storage_id", default_value="sqlite3"),
            DeclareLaunchArgument("bag_start_time_ns", default_value="0"),
            DeclareLaunchArgument("bag_preserve_record_order", default_value="false"),
            DeclareLaunchArgument("publish_rate_hz", default_value="100.0"),
            DeclareLaunchArgument("path_frame_id", default_value="basalt_world"),
            DeclareLaunchArgument("body_frame_id", default_value="basalt_body"),
            DeclareLaunchArgument("max_path_length", default_value="10000"),
            DeclareLaunchArgument("use_imu", default_value="true"),
            DeclareLaunchArgument("use_double", default_value="true"),
            DeclareLaunchArgument("use_camera_info_calibration", default_value="false"),
            DeclareLaunchArgument("camera_model", default_value="pinhole"),
            DeclareLaunchArgument("use_header_timestamps", default_value="true"),
            DeclareLaunchArgument("publish_debug_visuals", default_value="false"),
            DeclareLaunchArgument("publish_replay_inputs", default_value="false"),
            DeclareLaunchArgument("debug_visual_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("max_stereo_input_rate_hz", default_value="0.0"),
            DeclareLaunchArgument("use_global_position_factors", default_value="true"),
            DeclareLaunchArgument(
                "imu_startup_calibration_enabled", default_value="true"
            ),
            DeclareLaunchArgument("stereo_sync_queue_size", default_value="1000"),
            DeclareLaunchArgument("image_width", default_value="1280"),
            DeclareLaunchArgument("image_height", default_value="960"),
            DeclareLaunchArgument("fx", default_value="539.9363327026367"),
            DeclareLaunchArgument("fy", default_value="539.9363708496094"),
            DeclareLaunchArgument("cx", default_value="640.0"),
            DeclareLaunchArgument("cy", default_value="480.0"),
            DeclareLaunchArgument(
                "imu_to_cam_translation", default_value="[0.12, 0.03, 0.242]"
            ),
            DeclareLaunchArgument(
                "imu_to_cam_rotation_wxyz", default_value="[0.5, -0.5, 0.5, -0.5]"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=[
                    FindPackageShare("basalt_wrapper"),
                    "/rviz/basalt_wrapper.rviz",
                ],
            ),
            Node(
                package="basalt_wrapper",
                executable="basalt_node",
                name="basalt_node",
                output="screen",
                parameters=[
                    {
                        "left_image_topic": left_image_topic,
                        "right_image_topic": right_image_topic,
                        "imu_topic": imu_topic,
                        "calib_path": calib_path,
                        "config_path": config_path,
                        "trajectory_output_path": trajectory_output_path,
                        "ingest_log_path": ingest_log_path,
                        "diagnostics_log_path": diagnostics_log_path,
                        "marg_data_output_path": marg_data_output_path,
                        "accepted_input_bag_path": accepted_input_bag_path,
                        "accepted_input_bag_cache_mb": accepted_input_bag_cache_mb,
                        "global_position_topic": global_position_topic,
                        "input_mode": input_mode,
                        "bag_uri": bag_uri,
                        "bag_storage_id": bag_storage_id,
                        "bag_start_time_ns": bag_start_time_ns,
                        "bag_preserve_record_order": bag_preserve_record_order,
                        "publish_rate_hz": publish_rate_hz,
                        "path_frame_id": path_frame_id,
                        "body_frame_id": body_frame_id,
                        "max_path_length": max_path_length,
                        "use_imu": use_imu,
                        "use_double": use_double,
                        "use_camera_info_calibration": use_camera_info_calibration,
                        "camera_model": camera_model,
                        "use_header_timestamps": use_header_timestamps,
                        "publish_debug_visuals": publish_debug_visuals,
                        "publish_replay_inputs": publish_replay_inputs,
                        "debug_visual_rate_hz": debug_visual_rate_hz,
                        "max_stereo_input_rate_hz": max_stereo_input_rate_hz,
                        "use_global_position_factors": use_global_position_factors,
                        "imu_startup_calibration_enabled": imu_startup_calibration_enabled,
                        "stereo_sync_queue_size": stereo_sync_queue_size,
                        "image_width": image_width,
                        "image_height": image_height,
                        "fx": fx,
                        "fy": fy,
                        "cx": cx,
                        "cy": cy,
                        "imu_to_cam_translation": imu_to_cam_translation,
                        "imu_to_cam_rotation_wxyz": imu_to_cam_rotation_wxyz,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
