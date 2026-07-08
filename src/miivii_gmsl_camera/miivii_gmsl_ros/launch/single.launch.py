from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    enable_rviz = LaunchConfiguration('enable_rviz', default='false')
    pkg_share = get_package_share_directory('miivii_gmsl_camera')
    rviz_config_path = os.path.join(pkg_share, 'rviz_cfg', 'single.rviz')
    camera2_config_path = os.path.join(pkg_share, 'config', 'camera2_params.yaml')
    camera3_config_path = os.path.join(pkg_share, 'config', 'camera3_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='false',
            description='Enable RViz'
        ),

        Node(
            package='miivii_gmsl_camera',
            executable='miivii_gmsl_camera_node',
            name='miivii_gmsl_camera_node',
            output='log',
            parameters=[{
                'video0.active': False,
                'video0.camera_res': '1920x1080',
                'video0.output_res': '2880x1860',
                'video1.active': False,
                'video1.camera_res': '1920x1080',
                'video1.output_res': '960x540',
                'video2.active': False,
                'video2.camera_res': '2880x1860',
                'video2.output_res': '1280x1024',
                'video2.params_file': camera2_config_path,
                'video3.active': True,
                'video3.camera_res': '2880x1860',
                'video3.output_res': '2880x1860',
                'video3.params_file': camera3_config_path,
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],  
            condition=IfCondition(enable_rviz),
        )
    ])
