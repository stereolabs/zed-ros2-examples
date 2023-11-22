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

# ArUco Configurations to be loaded by detector Node
default_config_aruco = os.path.join(
    get_package_share_directory('zed_aruco_localization'),
    'config',
    'aruco_loc.yaml'
)

# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'urdf',
    'zed_descr.urdf.xacro'
)

def launch_setup(context, *args, **kwargs):
    wrapper_dir = get_package_share_directory('zed_wrapper')

    # Launch configuration variables
    svo_path = LaunchConfiguration('svo_path')

    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')

    zed_node_name = LaunchConfiguration('zed_node_name')
    aruco_node_name = LaunchConfiguration('aruco_node_name')

    config_common_path = LaunchConfiguration('config_path')
    config_path_aruco = LaunchConfiguration('config_path_aruco')

    serial_number = LaunchConfiguration('serial_number')

    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    xacro_path = LaunchConfiguration('xacro_path')
    gravity_alignment = LaunchConfiguration('gravity_alignment')

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
        get_package_share_directory('zed_aruco_localization'),
        'rviz2','aruco.rviz'
    )

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
            config_path_aruco,   # ArUco detector parameters
            # Overriding
            {
                'general.camera_name': camera_name_val,
                'general.camera_model': camera_model_val,
                'general.svo_file': svo_path,
                'general.serial_number': serial_number,
                'pos_tracking.publish_tf': publish_tf,
                'pos_tracking.publish_map_tf': publish_map_tf,
                'sensors.publish_imu_tf': publish_imu_tf,
                'pos_tracking.set_gravity_as_origin': gravity_alignment
            }
        ]
    )

    # ArUco processing component
    zed_aruco_component = ComposableNode(
        package='zed_aruco_localization',
        namespace=camera_name_val,
        plugin='stereolabs::ZedArucoLoc',
        name=aruco_node_name,
        parameters=[config_path_aruco],
        remappings=[
                ('in/zed_image', zed_node_name_val + '/rgb/image_rect_color'),
                ('in/camera_info', zed_node_name_val + '/rgb/camera_info'),
                ('set_pose', zed_node_name_val + '/set_pose')
            ]
    )

    # ROS 2 Component Container
    container = ComposableNodeContainer(
            name='zed_aruco_localization',
            namespace=camera_name_val,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                zed_wrapper_component,
                zed_aruco_component
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
                'aruco_node_name',
                default_value='aruco_node',
                description='The name of the ArUco detection node.'),
            DeclareLaunchArgument(
                'config_path',
                default_value=TextSubstitution(text=default_config_common),
                description='Path to the YAML configuration file for the camera.'),
            DeclareLaunchArgument(
                'config_path_aruco',
                default_value=TextSubstitution(text=default_config_aruco),
                description='Path to the YAML configuration file for the ArUco detector.'),
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
                'publish_tf',
                default_value='true',
                description='Enable publication of the `odom -> camera_link` TF.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_map_tf',
                default_value='true',
                description='Enable publication of the `map -> odom` TF. Note: Ignored if `publish_tf` is False.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_imu_tf',
                default_value='true',
                description='Enable publication of the IMU TF. Note: Ignored if `publish_tf` is False.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'gravity_alignment',
                default_value='false',
                description='Enable orientation alignment to the gravity vector. Note: if enabled the orientation of the markers must refer to Earth gravity.',
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
                description='Starts RVIZ2 preconfigured to show ArUco detection results',
                choices=['true', 'false']),
            OpaqueFunction(function=launch_setup)
        ]
    )
