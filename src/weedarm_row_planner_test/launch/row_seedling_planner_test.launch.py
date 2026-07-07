from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_speed', default_value='0.05'),
        DeclareLaunchArgument('safe_dist', default_value='0.04'),
        DeclareLaunchArgument('ground_z0', default_value='-0.13'),
        DeclareLaunchArgument('work_depth', default_value='0.02'),
        DeclareLaunchArgument('terrain_wave_amp', default_value='0.00'),
        DeclareLaunchArgument('first_up', default_value='true'),
        DeclareLaunchArgument('run_once', default_value='true'),
        DeclareLaunchArgument('safe_y', default_value='0.0'),
        DeclareLaunchArgument('safe_z', default_value='-0.08'),
        DeclareLaunchArgument('retract_time', default_value='2.0'),
        DeclareLaunchArgument('hold_safe_time', default_value='1.0'),
        DeclareLaunchArgument('auto_stop_cmd_start', default_value='true'),

        Node(
            package='weedarm_row_planner_test',
            executable='row_seedling_planner_test',
            name='row_seedling_planner_test',
            output='screen',
            parameters=[{
                'publish_rate_hz': 20.0,
                'point_count': 64,
                'point_dt': 0.02,

                'num_plants': 20,
                'spacing': 0.25,
                'jitter_y': 0.03,
                'leak_prob': 0.10,
                'replay_prob': 0.15,
                'replay_dist': 0.10,
                'cluster_dist': 0.10,

                'safe_dist': LaunchConfiguration('safe_dist'),
                'first_up': LaunchConfiguration('first_up'),
                'y_limit': 0.20,

                'vehicle_speed': LaunchConfiguration('vehicle_speed'),
                'loop_row': False,

                'ground_z0': LaunchConfiguration('ground_z0'),
                'work_depth': LaunchConfiguration('work_depth'),
                'terrain_slope_x': 0.0,
                'terrain_slope_y': 0.0,
                'terrain_wave_amp': LaunchConfiguration('terrain_wave_amp'),
                'terrain_wave_len': 1.0,
                'z_min': -0.40,
                'z_max': -0.02,

                'run_once': LaunchConfiguration('run_once'),
                'safe_y': LaunchConfiguration('safe_y'),
                'safe_z': LaunchConfiguration('safe_z'),
                'retract_time': LaunchConfiguration('retract_time'),
                'hold_safe_time': LaunchConfiguration('hold_safe_time'),
                'auto_stop_cmd_start': LaunchConfiguration('auto_stop_cmd_start'),

                'random_seed': 42,
                'preview_points': 300,
            }],
        )
    ])

