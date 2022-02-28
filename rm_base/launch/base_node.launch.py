import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    base_node = Node(
        package='rm_base',
        # namespace='rm_base',
        executable='simple_robot_base',
        name='test',
        output='screen',
        parameters=[
                {"serial_name": "/dev/ttyUSB0"},
                {"serial_send": True},
                {"serial_recv": False},
                {"serial_bps": 1152000},
                {"debug": True},
                {"custom_qos": True}
        ]
    )

    ld = LaunchDescription()
    ld.add_action(base_node)

    return ld