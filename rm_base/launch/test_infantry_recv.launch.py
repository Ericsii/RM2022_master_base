import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 用于下位机发送模式、子弹射速，上位机发送pitch、yaw云台控制，
    base_param_path = os.path.join(get_package_share_directory("rm_base"), "config/base_param.yaml")
    robot_name = 'sentry'
    base_node = Node(
            package='rm_base', 
            executable='robot_base',
            namespace= robot_name,
            name= 'robot_base',  
            parameters=[base_param_path],
            output='screen'
        )

    ld = LaunchDescription()
    ld.add_action(base_node)

    return ld