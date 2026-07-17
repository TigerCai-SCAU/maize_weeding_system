from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare("oid_wheel_encoder"), "config", "oid_wheel_encoder.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="oid_wheel_encoder",
                executable="wheel_encoder_node",
                name="oid_wheel_encoder",
                output="screen",
                parameters=[config],
            )
        ]
    )
