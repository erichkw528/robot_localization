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
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="roar_robot_localization",
                name="pose_to_pose_covariant_converter",
                executable="pose_converter_node",
                remappings=[
                    ("input_pose", "/gps/pose"),
                    ("output_pose_with_covariance", "gps/poseWithCov"),
                ],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("roar_robot_localization"),
                        "config",
                        "ekf.yaml",
                    )
                ],
            ),
            # launch_ros.actions.Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     name="swri_transform",
            #     arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            # ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("roar_robot_localization"),
                        "config",
                        "navsat_transform.yaml",
                    ),
                ],
                remappings=[("/gps/fix", "/gps/fix"), ("/imu", "/gps/imu")],
            ),
        ]
    )
