import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction
)
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # RVIZ2 Configurations to be loaded by ZED Node
    config_rviz2 = os.path.join(
        get_package_share_directory('zed_display_rviz2'), 'rviz2', 'zedx.rviz'
    )

    # RVIZ2 node
    rviz2_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='zedx_rviz2',
        output='screen',
        arguments=[['-d'], [config_rviz2]],
    )

    return [
        rviz2_node
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
