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
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    path_frame_id = LaunchConfiguration("path_frame_id")
    body_frame_id = LaunchConfiguration("body_frame_id")
    max_path_length = LaunchConfiguration("max_path_length")
    use_imu = LaunchConfiguration("use_imu")
    use_double = LaunchConfiguration("use_double")
    use_camera_info_calibration = LaunchConfiguration("use_camera_info_calibration")
    camera_model = LaunchConfiguration("camera_model")
    use_header_timestamps = LaunchConfiguration("use_header_timestamps")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    fx = LaunchConfiguration("fx")
    fy = LaunchConfiguration("fy")
    cx = LaunchConfiguration("cx")
    cy = LaunchConfiguration("cy")
    imu_to_cam_translation = LaunchConfiguration("imu_to_cam_translation")
    imu_to_cam_rotation_wxyz = LaunchConfiguration("imu_to_cam_rotation_wxyz")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("left_image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("right_image_topic", default_value=""),
            DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
            DeclareLaunchArgument("calib_path", default_value=""),
            DeclareLaunchArgument("config_path", default_value=""),
            DeclareLaunchArgument("publish_rate_hz", default_value="100.0"),
            DeclareLaunchArgument("path_frame_id", default_value="basalt_world"),
            DeclareLaunchArgument("body_frame_id", default_value="basalt_body"),
            DeclareLaunchArgument("max_path_length", default_value="10000"),
            DeclareLaunchArgument("use_imu", default_value="true"),
            DeclareLaunchArgument("use_double", default_value="false"),
            DeclareLaunchArgument("use_camera_info_calibration", default_value="false"),
            DeclareLaunchArgument("camera_model", default_value="pinhole"),
            DeclareLaunchArgument("use_header_timestamps", default_value="true"),
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
                        "publish_rate_hz": publish_rate_hz,
                        "path_frame_id": path_frame_id,
                        "body_frame_id": body_frame_id,
                        "max_path_length": max_path_length,
                        "use_imu": use_imu,
                        "use_double": use_double,
                        "use_camera_info_calibration": use_camera_info_calibration,
                        "camera_model": camera_model,
                        "use_header_timestamps": use_header_timestamps,
                        "image_width": image_width,
                        "image_height": image_height,
                        "fx": fx,
                        "fy": fy,
                        "cx": cx,
                        "cy": cy,
                        "imu_to_cam_translation": imu_to_cam_translation,
                        "imu_to_cam_rotation_wxyz": imu_to_cam_rotation_wxyz,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
