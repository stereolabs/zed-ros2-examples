# Copyright 2023 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command


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

    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf', 'zed_descr.urdf.xacro'
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
        namespace=camera_name,
        executable='robot_state_publisher',
        name='zed_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', camera_name, ' ',
                    'camera_model:=', camera_model
                ])
        }]
    )

    # ZED node using manual composition
    zed_node = Node(
        package='zed_rgb_convert',
        namespace=camera_name,
        executable='zed_rgb_convert',
        output='screen',
        parameters=[
            config_common,  # Common parameters
            config_camera,  # Camera related parameters
        ],
        remappings=[
            ("zed_image_4ch", "zed_node/rgb/image_rect_color"),
            ("camera_info", "zed_node/rgb/camera_info")
        ]
    )

    # Add nodes to LaunchDescription
    ld.add_action(rsp_node)
    ld.add_action(zed_node)

    return ld
