#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('seedling_semantic_mapping')
    default_config = os.path.join(pkg_share, 'config', 'seedling_pipeline.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Full path to seedling pipeline yaml config.'
    )
    config_file = LaunchConfiguration('config_file')
    localizer_arg = DeclareLaunchArgument(
        'localizer_executable',
        default_value='yolo_sep_localizer',
        description='Use yolo_sep_localizer or color_sep_localizer.'
    )
    localizer_executable = LaunchConfiguration('localizer_executable')

    return LaunchDescription([
        config_arg,
        localizer_arg,
        Node(
            package='seedling_semantic_mapping',
            executable='odom_tf_broadcaster',
            name='odom_tf_broadcaster',
            parameters=[config_file],
            output='screen',
        ),
        Node(
            package='seedling_semantic_mapping',
            executable='rolling_submap_builder',
            name='rolling_submap_builder',
            parameters=[config_file],
            output='screen',
        ),
        Node(
            package='seedling_semantic_mapping',
            executable=localizer_executable,
            name='yolo_sep_localizer',
            parameters=[config_file],
            output='screen',
        ),
        Node(
            package='seedling_semantic_mapping',
            executable='seedling_mapper',
            name='seedling_mapper',
            parameters=[config_file],
            output='screen',
        ),
    ])
