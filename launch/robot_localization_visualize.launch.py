from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base_path = Path(get_package_share_directory("roar_robot_localization"))

    rviz_path: Path = base_path / "params" / "localization.rviz"
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_path.as_posix()],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("roar_robot_localization"),
                        "launch",
                        "robot_localization.launch.py",
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("roar-gokart-urdf"),
                        "launch",
                        "state_publisher.launch.py",
                    )
                )
            ),
        ]
    )
