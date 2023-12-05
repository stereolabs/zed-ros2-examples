# Copyright 2023 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# ZED Configurations to be loaded by ZED Node
default_config_common = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'common.yaml'
)

# Depth Image to laser scan config file
default_config_cvt = os.path.join(
    get_package_share_directory('zed_depth_to_laserscan'),
    'config',
    'zed_depth_to_laserscan.yaml'
)

# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'urdf',
    'zed_descr.urdf.xacro'
)

def launch_setup(context, *args, **kwargs):

    # Launch configuration variables
    svo_path = LaunchConfiguration('svo_path')

    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')

    zed_node_name = LaunchConfiguration('zed_node_name')

    config_common_path = LaunchConfiguration('config_path')
    config_path_cvt = LaunchConfiguration('config_path_cvt')

    serial_number = LaunchConfiguration('serial_number')

    publish_urdf = LaunchConfiguration('publish_urdf')
    xacro_path = LaunchConfiguration('xacro_path')

    start_rviz = LaunchConfiguration('rviz')

    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    zed_node_name_val = zed_node_name.perform(context)

    if (camera_name_val == ''):
        camera_name_val = 'zed'

    config_camera_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        camera_model_val + '.yaml'
    )

    # Rviz2 Configurations to be loaded by ZED Node
    config_rviz2 = os.path.join(
        get_package_share_directory('zed_depth_to_laserscan'),
        'rviz2','zed_depth_to_laserscan.rviz'
    )

    # Camera depth frame for the laser scan topic
    camera_depth_frame = camera_name_val + '_left_camera_frame'

    # RVIZ2 node
    rviz2_node = Node(
        condition=IfCondition(start_rviz),
        package='rviz2',
        executable='rviz2',
        name=camera_model_val +'_rviz2',
        output='screen',
        arguments=[['-d'], [config_rviz2]],
    )

    # Robot State Publisher node
    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        namespace=camera_name_val,
        executable='robot_state_publisher',
        name='zed_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', camera_name_val, ' ',
                    'camera_model:=', camera_model_val, ' '
                ])
        }]
    )

    # ZED Wrapper component
    zed_wrapper_component = ComposableNode(
        package='zed_components',
        namespace=camera_name_val,
        plugin='stereolabs::ZedCamera',
        name=zed_node_name,
        parameters=[
            # YAML files
            config_common_path,  # Common parameters
            config_camera_path,  # Camera related parameters
            # Overriding
            {
                'general.camera_name': camera_name_val,
                'general.camera_model': camera_model_val,
                'general.svo_file': svo_path,
                'general.serial_number': serial_number
            }
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Depth to Laser scan component
    zed_cvt_component = ComposableNode(
        package='depthimage_to_laserscan',
        namespace=camera_name_val,
        plugin='depthimage_to_laserscan::DepthImageToLaserScanROS',
        name='depthimage_to_laserscan',
        parameters=[
            config_path_cvt,
            # Overriding
            {
                'output_frame': camera_depth_frame
            }],
        remappings=[
                ('depth', zed_node_name_val + '/depth/depth_registered'),
                ('depth_camera_info', zed_node_name_val + '/depth/camera_info')
            ],
        #extra_arguments=[{'use_intra_process_comms': True}] # Uncomment when supported by the package
    )

    # ROS 2 Component Container
    container = ComposableNodeContainer(
            name='zed_depth_to_laserscan',
            namespace=camera_name_val,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                zed_wrapper_component,
                zed_cvt_component
            ],
            output='screen',
    )

    return [
        rviz2_node,
        rsp_node,
        container
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
            DeclareLaunchArgument(
                'camera_name',
                default_value=TextSubstitution(text='zed'),
                description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.'),
            DeclareLaunchArgument(
                'camera_model',
                description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
                choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm']),
            DeclareLaunchArgument(
                'zed_node_name',
                default_value='zed_node',
                description='The name of the zed_wrapper node. All the topic will have the same prefix: `/<camera_name>/<zed_node_name>/`'),
            DeclareLaunchArgument(
                'config_path',
                default_value=TextSubstitution(text=default_config_common),
                description='Path to the YAML configuration file for the camera.'),
            DeclareLaunchArgument(
                'config_path_cvt',
                default_value=TextSubstitution(text=default_config_cvt),
                description='Path to the YAML configuration file for the detpth to laser scan conversion.'),
            DeclareLaunchArgument(
                'serial_number',
                default_value='0',
                description='The serial number of the camera to be opened.'),
            DeclareLaunchArgument(
                'publish_urdf',
                default_value='true',
                description='Enable URDF processing and starts Robot State Published to propagate static TF.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'xacro_path',
                default_value=TextSubstitution(text=default_xacro_path),
                description='Path to the camera URDF file as a xacro file.'),
            DeclareLaunchArgument(
                'svo_path',
                default_value=TextSubstitution(text='live'),
                description='Path to an input SVO file. Note: overrides the parameter `general.svo_file` in `common.yaml`.'),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                description='Starts RVIZ2 preconfigured to show conversion results',
                choices=['true', 'false']),
            OpaqueFunction(function=launch_setup)
        ]
    )
