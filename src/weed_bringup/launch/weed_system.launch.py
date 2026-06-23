from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    start_livox = LaunchConfiguration("start_livox")
    start_ahrs = LaunchConfiguration("start_ahrs")
    start_gnss_bridge = LaunchConfiguration("start_gnss_bridge")
    start_gnss_gate = LaunchConfiguration("start_gnss_gate")
    start_camera = LaunchConfiguration("start_camera")
    start_fast_livo = LaunchConfiguration("start_fast_livo")
    enable_gnss_constraint = LaunchConfiguration("enable_gnss_constraint")
    enable_wheel_constraint = LaunchConfiguration("enable_wheel_constraint")
    gnss_odom_topic = LaunchConfiguration("gnss_odom_topic")
    gnss_use_fixed_transform = LaunchConfiguration("gnss_use_fixed_transform")
    gnss_fixed_anchor_at_start = LaunchConfiguration("gnss_fixed_anchor_at_start")
    gnss_fixed_yaw_deg = LaunchConfiguration("gnss_fixed_yaw_deg")
    gnss_fixed_tx = LaunchConfiguration("gnss_fixed_tx")
    gnss_fixed_ty = LaunchConfiguration("gnss_fixed_ty")
    gnss_fixed_tz = LaunchConfiguration("gnss_fixed_tz")
    use_rviz = LaunchConfiguration("use_rviz")
    camera_rviz = LaunchConfiguration("camera_rviz")
    camera_log_level = LaunchConfiguration("camera_log_level")
    camera_output = LaunchConfiguration("camera_output")
    camera_verbose = LaunchConfiguration("camera_verbose")
    camera_log_timestamps = LaunchConfiguration("camera_log_timestamps")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_livox",
            default_value="true",
            description="Start Livox MID360 driver.",
        ),
        DeclareLaunchArgument(
            "start_ahrs",
            default_value="true",
            description="Start FDILink AHRS/GNSS driver.",
        ),
        DeclareLaunchArgument(
            "start_gnss_bridge",
            default_value="true",
            description="Start FDILink GNSS NED-to-ENU bridge.",
        ),
        DeclareLaunchArgument(
            "start_gnss_gate",
            default_value="true",
            description="Start GNSS quality gate for localization fusion.",
        ),
        DeclareLaunchArgument(
            "start_camera",
            default_value="true",
            description="Start Miivii GMSL camera driver.",
        ),
        DeclareLaunchArgument(
            "start_fast_livo",
            default_value="true",
            description="Start Fast-LIVO2 MID360 mapping.",
        ),
        DeclareLaunchArgument(
            "enable_gnss_constraint",
            default_value="false",
            description="Enable soft GNSS position constraint in Fast-LIVO2.",
        ),
        DeclareLaunchArgument(
            "enable_wheel_constraint",
            default_value="false",
            description="Enable soft wheel velocity constraint in Fast-LIVO2.",
        ),
        DeclareLaunchArgument(
            "gnss_odom_topic",
            default_value="/gnss/odom_gated",
            description="GNSS/INS odometry topic consumed by Fast-LIVO2.",
        ),
        DeclareLaunchArgument(
            "gnss_use_fixed_transform",
            default_value="true",
            description="Use calibrated planar GNSS-to-FastLIVO transform.",
        ),
        DeclareLaunchArgument(
            "gnss_fixed_anchor_at_start",
            default_value="true",
            description="Anchor fixed-yaw GNSS constraint at the first valid GNSS and Fast-LIVO poses.",
        ),
        DeclareLaunchArgument(
            "gnss_fixed_yaw_deg",
            default_value="86.963519",
            description="Fixed yaw from GNSS/INS odometry frame to Fast-LIVO frame, in degrees.",
        ),
        DeclareLaunchArgument(
            "gnss_fixed_tx",
            default_value="2.735186",
            description="Fixed X translation from GNSS/INS odometry frame to Fast-LIVO frame.",
        ),
        DeclareLaunchArgument(
            "gnss_fixed_ty",
            default_value="-0.386661",
            description="Fixed Y translation from GNSS/INS odometry frame to Fast-LIVO frame.",
        ),
        DeclareLaunchArgument(
            "gnss_fixed_tz",
            default_value="0.0",
            description="Fixed Z translation from GNSS/INS odometry frame to Fast-LIVO frame.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start Fast-LIVO2 RViz.",
        ),
        DeclareLaunchArgument(
            "camera_rviz",
            default_value="false",
            description="Start camera RViz.",
        ),
        DeclareLaunchArgument(
            "camera_log_level",
            default_value="warn",
            description="Miivii camera ROS log level.",
        ),
        DeclareLaunchArgument(
            "camera_output",
            default_value="log",
            description="Miivii camera output target: log or screen.",
        ),
        DeclareLaunchArgument(
            "camera_verbose",
            default_value="false",
            description="Print detailed Miivii camera configuration.",
        ),
        DeclareLaunchArgument(
            "camera_log_timestamps",
            default_value="false",
            description="Print throttled Miivii hardware timestamps.",
        ),
        include_launch(
            "livox_ros_driver2",
            "launch_ROS2",
            "msg_MID360_launch.py",
            condition=IfCondition(start_livox),
        ),

        include_launch(
            "fdilink_ahrs",
            "launch",
            "ahrs_driver.launch.py",
            condition=IfCondition(start_ahrs),
        ),

        include_launch(
            "miivii_gmsl_camera",
            "launch",
            "single.launch.py",
            condition=IfCondition(start_camera),
            launch_arguments={
                "enable_rviz": camera_rviz,
                "log_level": camera_log_level,
                "output": camera_output,
                "verbose_log": camera_verbose,
                "log_timestamps": camera_log_timestamps,
            },
        ),

        TimerAction(
            period=2.0,
            actions=[
                include_launch(
                    "fdilink_gnss_bridge",
                    "launch",
                    "gnss_bridge.launch.py",
                    condition=IfCondition(start_gnss_bridge),
                    launch_arguments={
                        "start_gnss_gate": start_gnss_gate,
                    },
                )
            ],
        ),

        TimerAction(
            period=5.0,
            actions=[
                include_launch(
                    "fast_livo",
                    "launch",
                    "mapping_mid360.launch.py",
                    condition=IfCondition(start_fast_livo),
                    launch_arguments={
                        "use_rviz": use_rviz,
                        "enable_gnss_constraint": enable_gnss_constraint,
                        "enable_wheel_constraint": enable_wheel_constraint,
                        "gnss_odom_topic": gnss_odom_topic,
                        "gnss_use_fixed_transform": gnss_use_fixed_transform,
                        "gnss_fixed_anchor_at_start": gnss_fixed_anchor_at_start,
                        "gnss_fixed_yaw_deg": gnss_fixed_yaw_deg,
                        "gnss_fixed_tx": gnss_fixed_tx,
                        "gnss_fixed_ty": gnss_fixed_ty,
                        "gnss_fixed_tz": gnss_fixed_tz,
                    },
                )
            ],
        ),
    ])
