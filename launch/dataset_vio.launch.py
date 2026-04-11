from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=[
                    FindPackageShare("basalt_wrapper"),
                    "/params/euroc_vio.params.yaml",
                ],
                description="Absolute path to a ROS parameter YAML for bag replay.",
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
                parameters=[config_file],
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
