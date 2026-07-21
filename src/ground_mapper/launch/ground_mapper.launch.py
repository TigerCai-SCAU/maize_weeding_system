#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("ground_mapper")
    default_config = os.path.join(
        package_share,
        "config",
        "ground_mapper.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to the ground mapper YAML file.",
            ),
            DeclareLaunchArgument(
                "map_mode",
                default_value="distance",
                description="rolling, distance, or global",
            ),
            DeclareLaunchArgument(
                "map_resolution_m",
                default_value="0.05",
                description="Persistent 2.5D map grid resolution.",
            ),
            DeclareLaunchArgument(
                "map_keep_behind_m",
                default_value="5.0",
                description="Distance retained behind the vehicle.",
            ),
            DeclareLaunchArgument(
                "map_keep_ahead_m",
                default_value="5.0",
                description="Distance retained ahead of the vehicle.",
            ),
            DeclareLaunchArgument(
                "map_max_cells",
                default_value="300000",
                description="Maximum number of persistent map cells.",
            ),
            DeclareLaunchArgument(
                "map_publish_rate_hz",
                default_value="0.5",
                description="Persistent-map PointCloud2 publish rate.",
            ),
            DeclareLaunchArgument(
                "save_map_on_shutdown",
                default_value="false",
                description="Save the persistent CSV map on Ctrl-C.",
            ),
            DeclareLaunchArgument(
                "save_map_path",
                default_value="/tmp/ground_global_map.csv",
                description="Persistent ground-map CSV output path.",
            ),
            Node(
                package="ground_mapper",
                executable="ground_mapper_node",
                name="ground_mapper_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "map_mode": LaunchConfiguration("map_mode"),
                        "map_resolution_m": ParameterValue(
                            LaunchConfiguration("map_resolution_m"),
                            value_type=float,
                        ),
                        "map_keep_behind_m": ParameterValue(
                            LaunchConfiguration("map_keep_behind_m"),
                            value_type=float,
                        ),
                        "map_keep_ahead_m": ParameterValue(
                            LaunchConfiguration("map_keep_ahead_m"),
                            value_type=float,
                        ),
                        "map_max_cells": ParameterValue(
                            LaunchConfiguration("map_max_cells"),
                            value_type=int,
                        ),
                        "map_publish_rate_hz": ParameterValue(
                            LaunchConfiguration("map_publish_rate_hz"),
                            value_type=float,
                        ),
                        "save_map_on_shutdown": ParameterValue(
                            LaunchConfiguration("save_map_on_shutdown"),
                            value_type=bool,
                        ),
                        "save_map_path": LaunchConfiguration(
                            "save_map_path"
                        ),
                    },
                ],
            ),
        ]
    )
