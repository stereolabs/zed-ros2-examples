"""Launch a ZED component and a RGB converter component
   in a component container."""

import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Define LaunchDescription variable
    ld = LaunchDescription()

    # use:
    #  - 'zed' for "ZED" camera
    #  - 'zedm' for "ZED mini" camera
    #  - 'zed2' for "ZED2" camera
    camera_model = 'zed2'

    # Camera name
    camera_name = 'zed2'

    # URDF file to be loaded by Robot State Publisher
    urdf = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf', camera_model + '.urdf'
    )

    # ZED Configurations to be loaded by ZED Node
    config_common = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        'common.yaml'
    )

    config_camera = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        camera_model + '.yaml'
    )

    # Set LOG format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'

    # Robot State Publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        node_namespace="/"+camera_name,
        node_executable='robot_state_publisher',
        node_name=camera_name+'_state_publisher',
        output='screen',
        arguments=[urdf],
    )

    zed_node_comp = ComposableNode(
        package='zed_components',
        node_plugin='stereolabs::ZedCamera',
        node_namespace='/'+camera_name,
        node_name='zed_node',
        parameters=[
            config_common,  # Common parameters
            config_camera,  # Camera related parameters
        ]
    )

    zed_cvt_node_comp = ComposableNode(
        package='zed_rgb_convert_component',
        node_plugin='stereolabs::ZedRgbCvtComponent',
        node_namespace='/'+camera_name,
        node_name='zed_cvt_node',
        remappings=[
            ('/'+camera_name+"/zed_image_4ch", '/'+camera_name+"/zed_node/rgb/image_rect_color"),
            ('/'+camera_name+"/camera_info", '/'+camera_name+"/zed_node/rgb/camera_info")
        ]
    )

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        package='rclcpp_components',
        node_namespace="/"+camera_name,
        node_name='zed_container',
        output='screen',
        node_executable='component_container',
        composable_node_descriptions=[
            zed_node_comp,
            zed_cvt_node_comp,
        ],
        parameters=[
            config_common,  # Common parameters
            config_camera,  # Camera related parameters
        ]
    )

    # Add nodes to LaunchDescription
    ld.add_action(rsp_node)
    ld.add_action(container)

    return ld
