import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    param_path = os.path.join(get_package_share_directory("rm_cam"), "config/cam_param.yml")

    mv_cam_node = Node(
        package="rm_cam",
        executable="mindvision_cam",
        name="mindvision_camera",
        parameters=[param_path],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(mv_cam_node)

    return ld