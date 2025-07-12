from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    monitor_config = os.path.join(
        get_package_share_directory('relocalization'),
        'config',
        'monitor_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='relocalization',
            executable='monitor',
            name='monitor',
            output='screen',
            parameters=[{'config_file': monitor_config},{'use_sim_time': False}]
        ),
    ])