from os import name
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rm_base',
            # namespace='rm_base',
            executable='simple_robot_base',
            name = 'send',
            output = 'screen',  
            parameters=[
                {"serial_name": "/dev/ttyUSB1"},
                {"serial_send": True},
                {"serial_recv": False},
                {"serial_bps": 1152000},
                {"debug": True}
            ]
        )
    ])