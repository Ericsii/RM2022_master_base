from os import name
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rm_base',
            # namespace='rm_base',
            executable='simple_robot_base',
            name = 'recv',
            output = 'screen',
            parameters=[
                {"serial_name": "/dev/pts/10"},
                {"serial_send": False},
                {"serial_recv": True}
            ]
        )
    ])