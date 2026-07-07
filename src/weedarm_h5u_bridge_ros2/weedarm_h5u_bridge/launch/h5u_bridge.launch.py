from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    plc_ip = LaunchConfiguration('plc_ip')
    return LaunchDescription([
        DeclareLaunchArgument('plc_ip', default_value='192.168.1.88'),
        Node(
            package='weedarm_h5u_bridge',
            executable='h5u_csp_bridge',
            name='h5u_csp_bridge',
            output='screen',
            parameters=[{
                'plc_ip': plc_ip,
                'plc_port': 502,
                'unit': 1,
                'addr_offset': 0,
                'word_order': 'lo_hi',
                'write_rate_hz': 20.0,
                'point_count': 64,
                'point_dt_ticks': 5,
                'lookahead_ticks': 50,
                'timeout_ticks': 125,
                'plc_tick_sec': 0.004,
                'trajectory_timeout_sec': 0.30,
                'use_fallback_when_no_traj': False,
                'y_limit_m': 0.20,
                'z_min_m': -0.40,
                'z_max_m': -0.02,
            }],
            remappings=[
                ('trajectory_yz', '/weedarm/trajectory_yz'),
                ('joint_state_feedback', '/weedarm/joint_state_feedback'),
                ('tool_yz_feedback', '/weedarm/tool_yz_feedback'),
                ('diagnostics', '/weedarm/diagnostics'),
                ('bridge_enable', '/weedarm/bridge_enable'),
            ],
        )
    ])
