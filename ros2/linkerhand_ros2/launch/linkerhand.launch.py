from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("linkerhand_ros2")
    params_file = os.path.join(pkg_share, "config", "linkerhand_params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("side", default_value="left"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("publish_rate", default_value="100.0"),
            DeclareLaunchArgument("enable_force_sensors", default_value="true"),
            Node(
                package="linkerhand_ros2",
                executable="linkerhand_node",
                name="linkerhand_node",
                namespace=LaunchConfiguration("side"),
                parameters=[
                    params_file,
                    {
                        "side": LaunchConfiguration("side"),
                        "can_interface": LaunchConfiguration("can_interface"),
                        "publish_rate": ParameterValue(
                            LaunchConfiguration("publish_rate"), value_type=float
                        ),
                        "enable_force_sensors": ParameterValue(
                            LaunchConfiguration("enable_force_sensors"), value_type=bool
                        ),
                    },
                ],
                output="screen",
            ),
        ]
    )
