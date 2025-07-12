from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcd_save',  # 包名
            executable='pcd_saver_node',  # 可执行文件
            name='pcd_saver_node',  # 节点名称
            output='screen',  # 输出到终端
        ),
    ])
