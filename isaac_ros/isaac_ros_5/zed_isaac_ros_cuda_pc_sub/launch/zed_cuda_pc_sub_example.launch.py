# Copyright 2026 Stereolabs
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
    ComposableNodeContainer,
    LoadComposableNodes
)
from launch_ros.descriptions import (
    ComposableNode
)

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def launch_setup(context, *args, **kwargs):

    # Get the path to the example configuration file
    example_params_file = os.path.join(
        get_package_share_directory('zed_isaac_ros_cuda_pc_sub'),
        'config',
        'example_params.yaml'
    )

    # List of actions to be launched
    actions = []

    namespace_val = 'zed_isaac'
    camera_model = LaunchConfiguration('camera_model')
    topic_name = LaunchConfiguration('topic_name')

    # ROS 2 Component Containers
    # The ZED node and the subscriber run in two different containers: the ZED
    # data are shared with the subscriber through cross-process CUDA IPC.
    # The same-process CUDA IPC path of `cuda_buffer_backend` can deliver a
    # buffer already reused by the ZED node, which stops the container.
    container_name = 'zed_container'
    sub_container_name = 'cuda_sub_container'

    for name in [container_name, sub_container_name]:
        info = '* Starting Composable node container: ' + namespace_val + '/' + name
        actions.append(LogInfo(msg=TextSubstitution(text=info)))
        actions.append(ComposableNodeContainer(
            name=name,
            namespace=namespace_val,
            package='rclcpp_components',
            executable='component_container_mt',
            arguments=['--ros-args', '--log-level', 'info'],
            output='screen',
        ))

    # ZED Wrapper launch file
    # Note: rclcpp Intra-Process Communication delivers the messages without
    # the buffer backend negotiation, so the ZED node publishes CPU data to the
    # intra-process subscribers. IPC is disabled to always publish CUDA buffers.
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_model': camera_model,
            'container_name': container_name,
            'namespace': namespace_val,
            'enable_ipc': 'false',
            'ros_params_override_path': example_params_file
        }.items()
    )
    actions.append(zed_wrapper_launch)

    # Define the CUDA zero-copy subscriber component
    cuda_sub_node = ComposableNode(
        package='zed_isaac_ros_cuda_pc_sub',
        plugin='stereolabs::ZedCudaPcSubComponent',
        name='zed_cuda_pc_sub_component',
        namespace=namespace_val,
        remappings=[
            ('point_cloud', topic_name)
        ],
        parameters=[
            example_params_file
        ]
    )

    # Create the full name of the subscriber container
    container_full_name = namespace_val + '/' + sub_container_name

    # Load the subscriber node into its container
    load_sub_node = LoadComposableNodes(
        composable_node_descriptions=[cuda_sub_node],
        target_container=container_full_name
    )
    actions.append(load_sub_node)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'camera_model',
                description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
                choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'zedxnano', 'zedxhdr', 'zedxhdrmini', 'zedxhdrmax', 'virtual', 'zedxonegs', 'zedxone4k', 'zedxonehdr', 'zedxonecore']),
            DeclareLaunchArgument(
                'topic_name',
                default_value='/zed_isaac/zed/point_cloud/cloud_registered',
                description='Point cloud topic to subscribe to.'),
            OpaqueFunction(function=launch_setup)
        ]
    )
