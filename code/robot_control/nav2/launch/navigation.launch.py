import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的路径
    pkg_dir = get_package_share_directory('robot_navigation')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    
    # 创建launch描述
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_dir, 'maps', 'map.yaml'),
            description='Full path to map yaml file to load'
        ),
        
        # 启动Nav2
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='nav2_bringup',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
                {
                    'use_sim_time': use_sim_time,
                    'yaml_filename': map_yaml_file
                }
            ]
        ),
        
        # 启动RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'robot_nav.yaml')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])