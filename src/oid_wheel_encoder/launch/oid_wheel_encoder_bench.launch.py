from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [
            FindPackageShare("oid_wheel_encoder"),
            "config",
            "oid_wheel_encoder_bench.yaml",
        ]
    )
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Absolute path to the bench wheel/virtual odom YAML.",
            ),
            Node(
                package="oid_wheel_encoder",
                executable="wheel_encoder_node",
                name="oid_wheel_encoder",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="oid_wheel_encoder",
                executable="bench_virtual_odom",
                name="bench_virtual_odom",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
