from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from pathlib import Path


def generate_launch_description():
    base_path = Path(get_package_share_directory("roar_robot_localization"))

    rviz_path: Path = base_path / "config" / "waypoint_collection_visualization.rviz"

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="swri_transform",
                arguments=["0", "0", "0", "0", "0", "0", "gps", "map"],
            ),
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_path.as_posix()],
            ),
        ]
    )
