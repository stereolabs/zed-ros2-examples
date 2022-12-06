# Copyright 2022 Stereolabs
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

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
from launch_ros.actions import Node


def parse_array_param(param):
    str = param.replace('[', '')
    str = str.replace(']', '')
    arr = str.split(',')

    return arr


def launch_setup(context, *args, **kwargs):
    names = LaunchConfiguration('cam_names')
    models = LaunchConfiguration('cam_models')
    serials = LaunchConfiguration('cam_serials')
    poses = LaunchConfiguration('cam_poses')

    names_arr = parse_array_param(names.perform(context))
    models_arr = parse_array_param(models.perform(context))
    serials_arr = parse_array_param(serials.perform(context))
    poses_arr = parse_array_param(poses.perform(context))

    num_cams = len(names_arr)

    if (num_cams != len(models_arr)):
        return [
            LogInfo(msg=TextSubstitution(
                text="The size of the `models` param array must be equal to the size of `names`"))
        ]

    if (num_cams != len(serials_arr)):
        return [
            LogInfo(msg=TextSubstitution(
                text="The size of the `serials` param array must be equal to the size of `names`"))
        ]

    if (num_cams != (len(poses_arr)/6)):
        return [
            LogInfo(msg=TextSubstitution(
                text="The size of the `poses` param array must be equal to the size of `names`"))
        ]

    return []


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'cam_names',
                description='An array containing the names of the cameras of the rig, e.g. ["zed_front","zed_back","zed_left", "zed_right"]'),
            DeclareLaunchArgument(
                'cam_models',
                description='An array containing the names of the cameras of the rig, e.g. ["zed2i","zed2","zed2", "zed2i"]'),
            DeclareLaunchArgument(
                'cam_serials',
                description='An array containing the serial numbers of the cameras of the rig, e.g. ["3001234","2001234","2004321", "3004321"]'),
            DeclareLaunchArgument(
                'cam_poses',
                description='An array containing the poses of the cameras of the rig with respect to the base frame link, '
                'e.g. [[0.5,0.0,0.0,0.0,0.0,0.0],[0.0,0.2,0.0,0.0,1.571,0.0]],[0.0,-0.2,0.0,0.0,-1.571,0.0],[-0.5,0.0,0.0,0.0,3.141,0.0]]]'),
            OpaqueFunction(function=launch_setup)
        ]
    )
