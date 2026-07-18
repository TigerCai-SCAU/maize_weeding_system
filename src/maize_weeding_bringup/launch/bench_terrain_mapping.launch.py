#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    wheel_share = get_package_share_directory("oid_wheel_encoder")
    ground_share = get_package_share_directory("ground_mapper")

    wheel_launch = f"{wheel_share}/launch/oid_wheel_encoder_bench.launch.py"
    ground_config = f"{ground_share}/config/ground_mapper_bench.yaml"

    return LaunchDescription(
        [
            Node(
                package="maize_weeding_bringup",
                executable="livox_custom_to_pointcloud",
                name="livox_custom_to_pointcloud",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/livox/lidar",
                        "output_topic": "/bench/livox_points",
                        "frame_id": "camera_init",
                        "min_range_m": 0.8,
                        "max_range_m": 50.0,
                        "publish_every_n_scans": 1,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(wheel_launch),
            ),
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package="ground_mapper",
                        executable="ground_mapper_node",
                        name="ground_mapper_node",
                        output="screen",
                        parameters=[ground_config],
                    )
                ],
            ),
        ]
    )
