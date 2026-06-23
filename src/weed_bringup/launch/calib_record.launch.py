from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch(package, *path_parts, condition=None, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), *path_parts])
        ),
        condition=condition,
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    bag_name = LaunchConfiguration("bag_name")
    start_record = LaunchConfiguration("start_record")
    start_paths = LaunchConfiguration("start_paths")

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_name",
            default_value="calib_traj_bag",
            description="Output rosbag directory for calibration trajectories.",
        ),
        DeclareLaunchArgument(
            "start_record",
            default_value="true",
            description="Start ros2 bag record after bringup.",
        ),
        DeclareLaunchArgument(
            "start_paths",
            default_value="true",
            description="Publish live nav_msgs/Path tracks for RViz.",
        ),

        include_launch(
            "weed_bringup",
            "launch",
            "weed_system.launch.py",
            launch_arguments={
                "enable_gnss_constraint": "false",
                "gnss_odom_topic": "/gnss/odom_gated",
            },
        ),

        TimerAction(
            period=8.0,
            actions=[
                Node(
                    condition=IfCondition(start_paths),
                    package="weed_calibration_tools",
                    executable="odom_to_path",
                    name="rtk_path_builder",
                    parameters=[{
                        "odom_topic": "/gnss/odom",
                        "path_topic": "/rtk_path",
                        "frame_id": "map",
                    }],
                    output="screen",
                ),
                Node(
                    condition=IfCondition(start_paths),
                    package="weed_calibration_tools",
                    executable="odom_to_path",
                    name="rtk_gated_path_builder",
                    parameters=[{
                        "odom_topic": "/gnss/odom_gated",
                        "path_topic": "/rtk_gated_path",
                        "frame_id": "map",
                    }],
                    output="screen",
                ),
                Node(
                    condition=IfCondition(start_paths),
                    package="weed_calibration_tools",
                    executable="odom_to_path",
                    name="ins_ned_path_builder",
                    parameters=[{
                        "odom_topic": "/ins/odom_ned",
                        "path_topic": "/ins_ned_path",
                        "frame_id": "ned",
                    }],
                    output="screen",
                ),
                ExecuteProcess(
                    condition=IfCondition(start_record),
                    cmd=[
                        "ros2",
                        "bag",
                        "record",
                        "/aft_mapped_to_init",
                        "/ins/odom_ned",
                        "/gnss/odom",
                        "/gnss/odom_gated",
                        "/NED_odometry",
                        "/gps/fix",
                        "/imu",
                        "/rtk_path",
                        "/rtk_gated_path",
                        "/ins_ned_path",
                        "-o",
                        bag_name,
                    ],
                    output="screen",
                )
            ],
        ),
    ])
