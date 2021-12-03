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
                {"serial_name": "/dev/ttyUSB0"},
                {"serial_send": True},
                {"serial_recv": True},
                {"serial_bps": 1152000},
                {"debug": False},
                {"custom_qos": True}
            ]
        ),
        Node(
            package='rm_base',
            executable='get_mode_client',
            name = 'clientnode',
            output = 'screen'
        )
        # Node(
        #     package='rm_base',
        #     executable='subscriber',
        #     name = 'node1',
        #     output = 'screen'
        # )

    ])