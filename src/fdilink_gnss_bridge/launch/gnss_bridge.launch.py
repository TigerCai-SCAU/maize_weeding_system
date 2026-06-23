from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    start_gnss_gate = LaunchConfiguration("start_gnss_gate")
    gate_max_horizontal_stddev = LaunchConfiguration("gate_max_horizontal_stddev")
    gate_max_vertical_stddev = LaunchConfiguration("gate_max_vertical_stddev")
    gate_output_min_interval_sec = LaunchConfiguration("gate_output_min_interval_sec")
    max_imu_age_sec = LaunchConfiguration("max_imu_age_sec")
    max_fix_age_sec = LaunchConfiguration("max_fix_age_sec")

    gnss_bridge = Node(
        package="fdilink_gnss_bridge",
        executable="gnss_bridge_node",
        name="fdilink_gnss_bridge",
        output="screen",
        parameters=[{
            "input_fix_topic": "/gps/fix",
            "output_fix_topic": "/gnss/fix",
            "input_imu_topic": "/imu",
            "input_ned_odom_topic": "/NED_odometry",
            "output_ned_odom_topic": "/ins/odom_ned",
            "output_enu_odom_topic": "/gnss/odom",
            "navsat_frame_id": "navsat_link",
            "ned_odom_frame_id": "ned",
            "enu_odom_frame_id": "map",
            "odom_child_frame_id": "ins_imu",
            "max_imu_age_sec": ParameterValue(max_imu_age_sec, value_type=float),
            "max_fix_age_sec": ParameterValue(max_fix_age_sec, value_type=float),
            "fallback_fix_xy_stddev": 0.5,
            "fallback_fix_z_stddev": 1.0,
            "fallback_odom_xy_stddev": 0.5,
            "fallback_odom_z_stddev": 1.0,
            "fallback_velocity_stddev": 0.2,
            "fallback_orientation_stddev": 0.1,
            "use_input_fix_covariance": True,
        }],
    )

    gnss_gate = Node(
        package="fdilink_gnss_bridge",
        executable="gnss_gate_node",
        name="gnss_gate",
        output="screen",
        condition=IfCondition(start_gnss_gate),
        parameters=[{
            "input_fix_topic": "/gnss/fix",
            "input_odom_topic": "/gnss/odom",
            "output_odom_topic": "/gnss/odom_gated",
            "require_fix": True,
            "require_covariance": True,
            "max_fix_age_sec": ParameterValue(max_fix_age_sec, value_type=float),
            "max_horizontal_stddev": ParameterValue(gate_max_horizontal_stddev, value_type=float),
            "max_vertical_stddev": ParameterValue(gate_max_vertical_stddev, value_type=float),
            "max_xy_step": 0.5,
            "max_z_step": 1.0,
            "output_min_interval_sec": ParameterValue(gate_output_min_interval_sec, value_type=float),
            "min_stable_samples": 5,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_gnss_gate",
            default_value="true",
            description="Start GNSS quality gate node.",
        ),
        DeclareLaunchArgument(
            "gate_max_horizontal_stddev",
            default_value="0.3",
            description="Maximum allowed GNSS horizontal standard deviation in meters.",
        ),
        DeclareLaunchArgument(
            "gate_max_vertical_stddev",
            default_value="1.0",
            description="Maximum allowed GNSS vertical standard deviation in meters.",
        ),
        DeclareLaunchArgument(
            "gate_output_min_interval_sec",
            default_value="0.05",
            description="Minimum interval between accepted GNSS odometry messages.",
        ),
        DeclareLaunchArgument(
            "max_imu_age_sec",
            default_value="0.1",
            description="Maximum allowed age difference when supplementing odometry orientation from IMU.",
        ),
        DeclareLaunchArgument(
            "max_fix_age_sec",
            default_value="1.0",
            description="Maximum allowed age difference when supplementing odometry covariance from GNSS fix.",
        ),
        gnss_bridge,
        gnss_gate,
    ])
