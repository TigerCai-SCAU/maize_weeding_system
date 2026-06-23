#!/usr/bin/python3
# -- coding: utf-8 --**

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Find path
    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2.rviz")

    #这里我们修改加载的雷达参数配置文件：mid360.yaml
    avia_config_cmd = os.path.join(config_file_dir, "mid360.yaml")
    #相机内参配置文件保持不变
    camera_config_cmd = os.path.join(config_file_dir, "camera_pinhole.yaml")

    # 打开 use_rviz
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to launch Rviz2",
    )

    avia_config_arg = DeclareLaunchArgument(
        'avia_params_file',
        default_value=avia_config_cmd,
        description='Full path to the ROS2 parameters file to use for fast_livo2 nodes',
    )

    camera_config_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=camera_config_cmd,
        description='Full path to the ROS2 parameters file to use for vikit_ros nodes',
    )

    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn', 
        default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    enable_gnss_constraint_arg = DeclareLaunchArgument(
        'enable_gnss_constraint',
        default_value='false',
        description='Enable soft GNSS position constraint for Fast-LIVO2.')

    enable_wheel_constraint_arg = DeclareLaunchArgument(
        'enable_wheel_constraint',
        default_value='false',
        description='Enable soft wheel velocity constraint for Fast-LIVO2.')

    gnss_odom_topic_arg = DeclareLaunchArgument(
        'gnss_odom_topic',
        default_value='/gnss/odom_gated',
        description='GNSS/INS odometry topic used by Fast-LIVO2 GNSS constraint.')

    gnss_use_fixed_transform_arg = DeclareLaunchArgument(
        'gnss_use_fixed_transform',
        default_value='true',
        description='Use fixed planar GNSS-to-FastLIVO transform instead of online yaw alignment.')

    gnss_fixed_anchor_at_start_arg = DeclareLaunchArgument(
        'gnss_fixed_anchor_at_start',
        default_value='true',
        description='Anchor fixed-yaw GNSS constraint to the first valid GNSS and Fast-LIVO poses.')

    gnss_fixed_yaw_deg_arg = DeclareLaunchArgument(
        'gnss_fixed_yaw_deg',
        default_value='86.963519',
        description='Fixed yaw from GNSS/INS odometry frame to Fast-LIVO frame, in degrees.')

    gnss_fixed_tx_arg = DeclareLaunchArgument(
        'gnss_fixed_tx',
        default_value='2.735186',
        description='Fixed X translation from GNSS/INS odometry frame to Fast-LIVO frame.')

    gnss_fixed_ty_arg = DeclareLaunchArgument(
        'gnss_fixed_ty',
        default_value='-0.386661',
        description='Fixed Y translation from GNSS/INS odometry frame to Fast-LIVO frame.')

    gnss_fixed_tz_arg = DeclareLaunchArgument(
        'gnss_fixed_tz',
        default_value='0.0',
        description='Fixed Z translation from GNSS/INS odometry frame to Fast-LIVO frame.')

    avia_params_file = LaunchConfiguration('avia_params_file')
    camera_params_file = LaunchConfiguration('camera_params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    enable_gnss_constraint = LaunchConfiguration('enable_gnss_constraint')
    enable_wheel_constraint = LaunchConfiguration('enable_wheel_constraint')
    gnss_odom_topic = LaunchConfiguration('gnss_odom_topic')
    gnss_use_fixed_transform = LaunchConfiguration('gnss_use_fixed_transform')
    gnss_fixed_anchor_at_start = LaunchConfiguration('gnss_fixed_anchor_at_start')
    gnss_fixed_yaw_deg = LaunchConfiguration('gnss_fixed_yaw_deg')
    gnss_fixed_tx = LaunchConfiguration('gnss_fixed_tx')
    gnss_fixed_ty = LaunchConfiguration('gnss_fixed_ty')
    gnss_fixed_tz = LaunchConfiguration('gnss_fixed_tz')

    return LaunchDescription([
        use_rviz_arg,
        avia_config_arg,
        camera_config_arg,
        use_respawn_arg,
        enable_gnss_constraint_arg,
        enable_wheel_constraint_arg,
        gnss_odom_topic_arg,
        gnss_use_fixed_transform_arg,
        gnss_fixed_anchor_at_start_arg,
        gnss_fixed_yaw_deg_arg,
        gnss_fixed_tx_arg,
        gnss_fixed_ty_arg,
        gnss_fixed_tz_arg,

        Node(
            package="image_transport",
            executable="republish",
            name="republish",
            arguments=[ 
                'raw', 
                'raw',
            ],
            remappings=[
                ("in",  "/miivii_gmsl/image3"), 
                ("out", "/left_camera/image")
            ],
            output="screen",
            respawn=use_respawn,
        ),
        
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[
                avia_params_file,
                camera_params_file,
                {
                    "gnss_constraint.enable": ParameterValue(enable_gnss_constraint, value_type=bool),
                    "gnss_constraint.odom_topic": gnss_odom_topic,
                    "gnss_constraint.use_fixed_transform": ParameterValue(gnss_use_fixed_transform, value_type=bool),
                    "gnss_constraint.fixed_anchor_at_start": ParameterValue(gnss_fixed_anchor_at_start, value_type=bool),
                    "gnss_constraint.fixed_yaw_deg": ParameterValue(gnss_fixed_yaw_deg, value_type=float),
                    "gnss_constraint.fixed_translation_x": ParameterValue(gnss_fixed_tx, value_type=float),
                    "gnss_constraint.fixed_translation_y": ParameterValue(gnss_fixed_ty, value_type=float),
                    "gnss_constraint.fixed_translation_z": ParameterValue(gnss_fixed_tz, value_type=float),
                    "wheel_constraint.enable": ParameterValue(enable_wheel_constraint, value_type=bool),
                },
            ],
            output="screen"
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen"
        ),
    ])
