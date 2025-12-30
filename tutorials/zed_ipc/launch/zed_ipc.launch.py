# Copyright 2025 Stereolabs
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
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo
)
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
from launch_ros.actions import (
    LoadComposableNodes
)
from launch_ros.descriptions import ComposableNode

# Function to parse array parameters
def parse_array_param(param):
    str = param.replace('[', '')
    str = str.replace(']', '')
    str = str.replace(' ', '')
    arr = str.split(',')
    
    if arr[0] == '':
        return []
        
    return arr

def launch_setup(context, *args, **kwargs):

    # List of actions to be launched
    actions = []

    # Arguments
    names = LaunchConfiguration('cam_names')
    models = LaunchConfiguration('cam_models')
    serials = LaunchConfiguration('cam_serials')
    ids = LaunchConfiguration('cam_ids')
    disable_tf = LaunchConfiguration('disable_tf')

    # Call the multi-camera launch file
    multi_camera_launch_file = os.path.join(
        get_package_share_directory('zed_multi_camera'),
        'launch',
        'zed_multi_camera.launch.py'
    )
    zed_multi_camera = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(multi_camera_launch_file),
        launch_arguments={
            'cam_names': names,
            'cam_models': models,
            'cam_serials': serials,
            'cam_ids': ids,
            'disable_tf': disable_tf
        }.items()
    )
    actions.append(zed_multi_camera)
    
    cam_count = len(names.perform(context).split(','))

    # Create topic remappings for the point cloud node
    remappings = []
    name_array = parse_array_param(names.perform(context))
    for i in range(cam_count):
        base_topic = 'pointcloud_' + str(i)
        remap = '/zed_multi/' + name_array[i] + '/point_cloud/cloud_registered'
        remapping = (base_topic, remap)
        remappings.append(remapping)

    # Create the point cloud node
    pc_node = ComposableNode(
        package='zed_ipc',
        plugin='stereolabs::PointCloudComponent',
        name='ipc_user_node',
        namespace='zed_multi',
        parameters=[{
            'cam_count': cam_count
        }],
        remappings=remappings,
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Load the point cloud node in the container
    load_pc_node = LoadComposableNodes(
        composable_node_descriptions=[pc_node],
        target_container='/zed_multi/zed_multi_container'
    )
    actions.append(load_pc_node)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'cam_names',
                description='An array containing the names of the cameras, e.g. [zed_front,zed_back]'),
            DeclareLaunchArgument(
                'cam_models',
                description='An array containing the names of the cameras, e.g. [zed2i,zed2]'),
            DeclareLaunchArgument(
                'cam_serials',
                default_value=[],
                description='An array containing the serial numbers of the cameras, e.g. [35199186,23154724]'),
            DeclareLaunchArgument(
                'cam_ids',
                default_value=[],
                description='An array containing the ID number of the cameras, e.g. [0,1]'),
            DeclareLaunchArgument(
                'disable_tf',
                default_value='False',
                description='If `True` disable TF broadcasting for all the cameras in order to fuse visual odometry information externally.'),
            OpaqueFunction(function=launch_setup)
        ]
    )
