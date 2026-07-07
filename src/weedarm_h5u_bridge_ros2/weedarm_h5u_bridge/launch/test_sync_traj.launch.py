from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('amp_y', default_value='0.01'),
        DeclareLaunchArgument('amp_z', default_value='0.01'),
        DeclareLaunchArgument('z_center', default_value='-0.12'),
        DeclareLaunchArgument('period', default_value='4.0'),
        Node(
            package='weedarm_h5u_bridge',
            executable='sync_traj_publisher',
            name='sync_traj_publisher',
            output='screen',
            parameters=[{
                'amp_y': LaunchConfiguration('amp_y'),
                'amp_z': LaunchConfiguration('amp_z'),
                'z_center': LaunchConfiguration('z_center'),
                'period': LaunchConfiguration('period'),
                'publish_rate_hz': 20.0,
                'point_count': 64,
                'point_dt': 0.02,
            }],
            remappings=[
                ('trajectory_yz', '/weedarm/trajectory_yz'),
            ],
        )
    ])
