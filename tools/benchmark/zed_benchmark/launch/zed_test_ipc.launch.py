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

from sympy import use

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
    LaunchConfiguration
)
from launch_ros.actions import (
    LoadComposableNodes,
    Node
)
from launch_ros.descriptions import (
    ComposableNode
)

# Function to parse array parameters
def parse_array_param(param):
    str = param.replace('[', '')
    str = str.replace(']', '')
    str = str.replace(' ', '')
    arr = str.split(',')
    return arr

def launch_setup(context, *args, **kwargs):

    # List of actions to be launched
    actions = []

    # Arguments
    names = LaunchConfiguration('cam_names')
    models = LaunchConfiguration('cam_models')
    serials = LaunchConfiguration('cam_serials')
    disable_tf = LaunchConfiguration('disable_tf')
    use_ipc = LaunchConfiguration('use_ipc')

    use_ipc_val = use_ipc.perform(context)

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
            'disable_tf': disable_tf
        }.items()
    )
    actions.append(zed_multi_camera)
    
    cam_count = len(names.perform(context).split(','))

    
    # Start a benchmark node for each point cloud topic of each camera
    name_array = parse_array_param(names.perform(context))
    for i in range(cam_count):

        # Topic name to subscribe to
        topic_name = '/zed_multi/' + name_array[i] + '/point_cloud/cloud_registered'

        if( use_ipc_val=='True'):
            # Create the point cloud node
            benchmark_node = ComposableNode(
                package='zed_topic_benchmark_component',
                plugin='stereolabs::TopicBenchmarkComponent',
                name='benchmark_' + str(i),
                namespace='zed_multi',
                parameters=[{
                    'topic_name': topic_name,
                    'use_ros_log': True
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            )

            # Load the point cloud node in the container
            load_pc_node = LoadComposableNodes(
                composable_node_descriptions=[benchmark_node],
                target_container='/zed_multi/zed_multi_container'
            )
            actions.append(load_pc_node)
        else:
            # Launch each benchmark in a separate process
            benchmark_node = Node(
                package='zed_topic_benchmark',
                executable='zed_topic_benchmark',
                name='benchmark_' + str(i),
                namespace='zed_multi',
                output='screen',
                parameters=[{
                    'topic_name': topic_name,
                    'use_ros_log': True,
                    'avg_win_size': 5000
                }]
            )
            actions.append(benchmark_node)

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
                description='An array containing the serial numbers of the cameras, e.g. [35199186,23154724]'),
            DeclareLaunchArgument(
                'disable_tf',
                default_value='False',
                description='If `True` disable TF broadcasting for all the cameras in order to fuse visual odometry information externally.'),
            DeclareLaunchArgument(
                'use_ipc',
                default_value='True',
                description='If `True` load the benchmark nodes in the same ZED Camera container using IPC communication.'),
            OpaqueFunction(function=launch_setup)
        ]
    )
