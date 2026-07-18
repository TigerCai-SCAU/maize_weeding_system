#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("fast_livo"),
        "config",
    )
    rviz_config_file = os.path.join(
        get_package_share_directory("fast_livo"),
        "rviz_cfg",
        "fast_livo2.rviz",
    )

    mid360_config_file = os.path.join(config_dir, "mid360.yaml")
    camera_config_file = os.path.join(config_dir, "camera_pinhole.yaml")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Whether to launch RViz2.",
    )
    mid360_config_arg = DeclareLaunchArgument(
        "avia_params_file",
        default_value=mid360_config_file,
        description="FAST-LIVO MID360 parameter file.",
    )
    camera_config_arg = DeclareLaunchArgument(
        "camera_params_file",
        default_value=camera_config_file,
        description="Camera model parameter file.",
    )

    avia_params_file = LaunchConfiguration("avia_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")

    parameter_blackboard = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="parameter_blackboard",
        parameters=[camera_params_file],
        output="screen",
    )

    fast_livo_node = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laserMapping",
        parameters=[avia_params_file],
        output="screen",
    )

    # The ROS2 camera loader waits only briefly for the remote parameter
    # service. Five seconds makes parameter discovery deterministic on Jetson.
    delayed_fast_livo = TimerAction(
        period=5.0,
        actions=[fast_livo_node],
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            mid360_config_arg,
            camera_config_arg,
            parameter_blackboard,
            delayed_fast_livo,
            rviz_node,
        ]
    )
