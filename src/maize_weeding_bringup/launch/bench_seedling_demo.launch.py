#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("maize_weeding_bringup")
    seedling_share = get_package_share_directory("seedling_semantic_mapping")

    terrain_launch = f"{bringup_share}/launch/bench_terrain_mapping.launch.py"
    seedling_launch = f"{seedling_share}/launch/seedling_pipeline.launch.py"
    seedling_config = f"{seedling_share}/config/seedling_pipeline_bench.yaml"

    localizer = LaunchConfiguration("localizer_executable")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "localizer_executable",
                default_value="color_sep_localizer",
                description="color_sep_localizer for targets or yolo_sep_localizer.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(terrain_launch),
            ),
            TimerAction(
                period=3.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(seedling_launch),
                        launch_arguments={
                            "config_file": seedling_config,
                            "localizer_executable": localizer,
                        }.items(),
                    )
                ],
            ),
        ]
    )
