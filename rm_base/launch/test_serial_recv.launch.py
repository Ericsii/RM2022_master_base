from os import name
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rm_base',
            # namespace='rm_base',
            executable='simple_robot_base',
            name = 'up',          #用于下位机发送模式、子弹射速，上位机发送pitch、yaw云台控制，
            output = 'screen',
            parameters=[
                {"serial_name": "/dev/ttyUSB2"},   #ttyUSB0
                {"serial_send": False},
                {"serial_recv": True},
                {"serial_bps": 1152000},
                {"debug": True},
                {"custom_qos": True}
            ]
        ),
        # Node(
        #     package='rm_base',
        #     executable='simple_robot_base',
        #     name = 'pose_stamp',   #用于下位机发送姿态，通信阶段同步姿态帧与相机帧
        #     output = 'screen',
        #     parameters=[
        #         {"serial_name": "/dev/ttyUSB1"},   #ttyUSB0
        #         {"serial_send": True},
        #         {"serial_recv": True},
        #         {"serial_bps": 1152000},
        #         {"debug": False},
        #         {"custom_qos": True}
        #     ]
        # )
    ])