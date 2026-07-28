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
    disable_tf = LaunchConfiguration('disable_tf')
    topic_name = LaunchConfiguration('topic_name')
    use_ipc = LaunchConfiguration('use_ipc')
    subscription_mode = LaunchConfiguration('subscription_mode')
    qos_reliability = LaunchConfiguration('qos_reliability')
    qos_durability = LaunchConfiguration('qos_durability')
    qos_history = LaunchConfiguration('qos_history')
    qos_depth = LaunchConfiguration('qos_depth')
    avg_win_size = LaunchConfiguration('avg_win_size')
    test_duration_sec = LaunchConfiguration('test_duration_sec')
    test_sample_count = LaunchConfiguration('test_sample_count')
    log_file_path = LaunchConfiguration('log_file_path')

    use_ipc_val = use_ipc.perform(context)
    subscription_mode_val = subscription_mode.perform(context)
    qos_reliability_val = qos_reliability.perform(context)
    qos_durability_val = qos_durability.perform(context)
    qos_history_val = qos_history.perform(context)
    qos_depth_val = int(qos_depth.perform(context))
    avg_win_size_val = int(avg_win_size.perform(context))
    topic_name_val = topic_name.perform(context)
    test_duration_sec_val = float(test_duration_sec.perform(context))
    test_sample_count_val = int(test_sample_count.perform(context))
    log_file_path_val = log_file_path.perform(context)

    # Call the multi-camera launch file
    multi_camera_launch_file = os.path.join(
        get_package_share_directory('zed_multi_camera'),
        'launch',
        'zed_multi_camera.launch.py'
    )
    zed_multi_camera = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            multi_camera_launch_file),
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
        topic_name_full = '/zed_multi/' + name_array[i] + topic_name_val

        # Derive a per-camera log file so concurrent benchmark nodes do not
        # overwrite each other (e.g. report.txt -> report_zed_front.txt).
        log_file_path_full = ''
        if log_file_path_val != '':
            base, ext = os.path.splitext(log_file_path_val)
            log_file_path_full = base + '_' + name_array[i] + ext

        # Parameters shared by both branches, kept in one place on purpose:
        # when each branch carried its own copy they drifted apart, and a
        # use_ipc:=True run then measured something subtly different from a
        # use_ipc:=False one, which makes the comparison worthless.
        # The `qos.*` names are the node's own parameters, effective on every
        # subscription path (unlike ROS 2's qos_overrides.*, which a generic
        # subscription never reads).
        benchmark_params = {
            'topic_name': topic_name_full,
            'subscription_mode': subscription_mode_val,
            'qos.reliability': qos_reliability_val,
            'qos.durability': qos_durability_val,
            'qos.history': qos_history_val,
            'qos.depth': qos_depth_val,
            'avg_win_size': avg_win_size_val,
            'use_ros_log': True,
            'test_duration_sec': test_duration_sec_val,
            'test_sample_count': test_sample_count_val,
            'log_file_path': log_file_path_full
        }

        if (use_ipc_val == 'True'):
            # Load the benchmark as a component in the camera container with
            # intra-process comms enabled. 'subscription_mode' must resolve to
            # a typed subscription for that path to be usable at all: rclcpp
            # only registers the templated rclcpp::Subscription<T> with the
            # IntraProcessManager, never a runtime-typed GenericSubscription.
            benchmark_node = ComposableNode(
                package='zed_topic_benchmark_component',
                plugin='stereolabs::TopicBenchmarkComponent',
                name='benchmark_' + str(i),
                namespace='zed_multi',
                parameters=[benchmark_params],
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
                parameters=[benchmark_params]
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
                description='If `True` load the benchmark nodes as components in the same ZED Camera container, instead of starting them as separate processes. Note: despite the name this does NOT yield an Intra Process Communication measurement, because a generic (runtime-typed) subscription never takes the intra-process path in rclcpp. See the package README.'),
            DeclareLaunchArgument(
                'subscription_mode',
                default_value='typed',
                description='Subscription path of the benchmark nodes: `auto`, `generic` or `typed`. Only `typed` can take the intra-process path. Applied to both branches so that `use_ipc:=True`/`False` stay comparable.'),
            DeclareLaunchArgument(
                'qos_reliability',
                default_value='best_effort',
                description='Subscriber QoS reliability: `best_effort` or '
                            '`reliable`. Note that a Reliable subscriber '
                            'cannot match a Best Effort publisher, which is '
                            'how ZED image and cloud topics are published.'),
            DeclareLaunchArgument(
                'qos_durability',
                default_value='volatile',
                description='Subscriber QoS durability: `volatile` or '
                            '`transient_local`.'),
            DeclareLaunchArgument(
                'qos_history',
                default_value='keep_last',
                description='Subscriber QoS history: `keep_last` or '
                            '`keep_all`.'),
            DeclareLaunchArgument(
                'qos_depth',
                default_value='1',
                description='Subscriber QoS depth, used by `keep_last`. '
                            'Must be >= 1.'),
            DeclareLaunchArgument(
                'avg_win_size',
                default_value='500',
                description='Window size of the running averages. Applied to '
                            'both branches: the report min/max are tracked on '
                            'the windowed average, so an asymmetric window '
                            'would make use_ipc:=True/False incomparable.'),
            DeclareLaunchArgument(
                'topic_name',
                default_value='/point_cloud/cloud_registered',
                description='The name of the topic to benchmark, without prefix.'),
            DeclareLaunchArgument(
                'test_duration_sec',
                default_value='0.0',
                description='Duration of the benchmark test in seconds. `0.0` runs until interrupted with Ctrl+C.'),
            DeclareLaunchArgument(
                'test_sample_count',
                default_value='0',
                description='Number of messages to acquire before stopping the test. `0` runs until interrupted with Ctrl+C.'),
            DeclareLaunchArgument(
                'log_file_path',
                default_value='',
                description='Path of the file where the benchmark report is saved. Empty disables file logging. With multiple cameras the camera name is appended to the file name to keep the reports separate.'),
            OpaqueFunction(function=launch_setup)
        ]
    )
