import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rm_base'), 'launch'),
            '/base_node.launch.py'])
        )
    mv_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rm_entity_cam'), 'launch'),
            '/mindvision_cam.launch.py'])
        )
    auto_aim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rm_auto_aim'), 'launch'),
            '/auto_aim_node.launch.py'])
        )

    robot_name = 'infantry4'
    namespace_launch1 = GroupAction(
        actions=[
            PushRosNamespace(robot_name),
            base_launch,
        ]
    )
    namespace_launch2 = GroupAction(
        actions=[
            PushRosNamespace(robot_name),
            mv_cam_launch,
        ]
    )
    namespace_launch3 = GroupAction(
        actions=[
            PushRosNamespace(robot_name),
            auto_aim_launch
        ]
    )
    return LaunchDescription([
        namespace_launch1,
        namespace_launch2,
        namespace_launch3
    ])
    # return LaunchDescription([
    #     base_launch,
    #     mv_cam_launch,
    #     auto_aim_launch
    # ])
