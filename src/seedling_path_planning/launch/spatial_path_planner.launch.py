#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("seedling_path_planning")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=f"{share}/config/spatial_path_planner.yaml",
            ),
            Node(
                package="seedling_path_planning",
                executable="spatial_path_planner",
                name="spatial_path_planner",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
        ]
    )
