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
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    Command
)
from launch.actions import (
    DeclareLaunchArgument
)
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable
)

# Cmaera name and model
camera_name_1 = 'zed_front'
camera_name_2 = 'zed_rear'
camera_model_1 = 'zedx'
camera_model_2 = 'zedxm'

# Rviz2 Configurations to be loaded by ZED Node
config_rviz2 = os.path.join(
    get_package_share_directory('zed_robot_integration'),
    'rviz2','view_config.rviz'
)

# URDF/xacro file to be loaded by the Robot State Publisher node
xacro_path = os.path.join(
    get_package_share_directory('zed_robot_integration'),
    'urdf',
    'zedbot_scoutm_dual.urdf.xacro'
)

def launch_setup(context, *args, **kwargs):
     # Launch configuration variables
    use_zed_localization = LaunchConfiguration('use_zed_localization')


    # Robot URDF from xacro
    robot_description = Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name_1:=', camera_name_1, ' ',
                    'camera_name_2:=', camera_name_2, ' ',
                    'camera_model_1:=', camera_model_1, ' ',
                    'camera_model_2:=', camera_model_2, ' ',
                    'use_zed_localization:=', use_zed_localization, 
                ])

    # RVIZ2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[['-d'], [config_rviz2]],
    )

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='scoutm_robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description
            }
        ])

    # Joint State Publisher
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='scoutm_joint_state_publisher'
    )

    return [
        rviz2_node,
        rsp_node,
        jsp_node
    ]

def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
            DeclareLaunchArgument(
                'use_zed_localization',
                default_value='true',
                description='Creates a TF tree with `camera_link` as root frame if `true`, otherwise the root is `base_ling`.',
                choices=['true', 'false']),
            OpaqueFunction(function=launch_setup)    
        ]
    )