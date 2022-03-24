from os import name
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rm_base',
            # namespace='rm_base',
            executable='simple_robot_base',
            name = 'sentry',
            output = 'screen',  
            parameters=[
                {"serial_name": "/dev/ttyUSB1"},
                {"serial_send": True},
                {"serial_recv": False},
                {"serial_bps": 1152000},
                {"debug": False},
                {"custom_qos": False}
            ]
        ),
        Node(
            package='rm_base',
            # namespace='rm_base',
            executable='simple_robot_base',
            name = 'up',          #用于下位机发送模式、子弹射速，上位机发送pitch、yaw云台控制，
            output = 'screen',
            parameters=[
                {"serial_name": "/dev/ttyUSB0"},   #ttyUSB0
                {"serial_send": False},
                {"serial_recv": True},
                {"serial_bps": 1152000},
                {"debug": False},
                {"custom_qos": False}
            ]
        )
    ])