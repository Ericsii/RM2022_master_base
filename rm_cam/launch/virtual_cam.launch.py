import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    video_path = os.path.join(get_package_share_directory("rm_cam"), "resource", "video2.mp4")

    virtual_cam_node = Node(
        package="rm_cam",
        executable="virtual_cam",
        name="virtual_video_camera",
        parameters=[{
            "video_path": video_path,
            'camera_name': 'virtual_camera',
            'camera_k': [1552.7, 0.0, 640.0, 0.0, 1537.9, 360.0, 0.0, 0.0, 1.0],
            'camera_d': [0.0, 0.0, 0.0, 0.0, 0.0],
            'fps': 20,
            "custom_qos": True
        }],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(virtual_cam_node)

    return ld