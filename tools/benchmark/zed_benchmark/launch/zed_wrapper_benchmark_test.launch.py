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
import yaml

from sympy import use

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch
import launch_ros.actions

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
    config_path = LaunchConfiguration('ros_params_override_path')
    disable_tf = LaunchConfiguration('disable_tf')
    use_ipc = LaunchConfiguration('use_ipc')
    results_file_path = LaunchConfiguration('results_file_path')
    performance_test_duration = LaunchConfiguration('performance_test_duration')
    
    # Retrieve important configuration information
    
    duration = performance_test_duration.perform(context)
    config_file = config_path.perform(context)

    depth_mode = 'NONE'
    use_od = False
    use_positional_tracking = False
    use_pointcloud = False
    
    if config_file and os.path.isfile(config_file):
    	with open(config_file, 'r') as f:
           cfg = yaml.safe_load(f) or {}
           depth_mode = (cfg.get('/**', {}).get('ros__parameters', {}).get('depth', {}).get('depth_mode', 'NONE'))
           use_od = (cfg.get('/**', {}).get('ros__parameters', {}).get('object_detection', {}).get('od_enabled', False))
           use_positional_tracking = (cfg.get('/**', {}).get('ros__parameters', {}).get('pos_tracking', {}).get('pos_tracking_enabled', False))
           use_pointcloud = (str(depth_mode).upper() != 'NONE')
   
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
    
    # CAPTURE -- Start a benchmark node for each image topic of each camera
    name_array = parse_array_param(names.perform(context))
    for i in range(cam_count):

        # Topic name to subscribe to
        topic_name = '/zed_multi/' + name_array[i] + '/rgb/color/rect/image'

        if( use_ipc_val=='True'):
            
            benchmark_node = ComposableNode(
                package='zed_benchmark_components',
                plugin='stereolabs::TopicBenchmarkComponent',
                name='benchmark_' + str(i),
                namespace='zed_multi',
                parameters=[{
                    'topic_name': topic_name,
                    'use_ros_log': True,
                    'results_file_path': results_file_path,
                    'avg_win_size': 5000
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            )

            
            load_capture_node = LoadComposableNodes(
                composable_node_descriptions=[benchmark_node],
                target_container='/zed_multi/zed_multi_container'
            )
            actions.append(load_capture_node)
        else:
           
            benchmark_node = Node(
                package='zed_topic_benchmark',
                executable='zed_topic_benchmark',
                name='benchmark_' + str(i),
                namespace='zed_multi',
                output='screen',
                parameters=[{
                    'topic_name': topic_name,
                    'use_ros_log': True,
                    'results_file_path': results_file_path,
                    'avg_win_size': 5000
                }]
            )
            actions.append(benchmark_node)


    
    # DEPTH -- Start a benchmark node for each point cloud topic of each camera (only if use_pointcloud is True)
    if use_pointcloud:
    	name_array = parse_array_param(names.perform(context))
    	for i in range(cam_count):

        	# Topic name to subscribe to
        	topic_name = '/zed_multi/' + name_array[i] + '/point_cloud/cloud_registered'

        	if( use_ipc_val=='True'):
            		
            		benchmark_node = ComposableNode(
                		package='zed_benchmark_components',
                		plugin='stereolabs::TopicBenchmarkComponent',
                		name='benchmark_pointcloud' + str(i),
                		namespace='zed_multi',
                		parameters=[{
                    		'topic_name': topic_name,
                    		'use_ros_log': True,
                    		'results_file_path': results_file_path,
                    		'avg_win_size': 5000
                		}],
                		extra_arguments=[{'use_intra_process_comms': True}]
            		)

            		
            		load_pc_node = LoadComposableNodes(
                		composable_node_descriptions=[benchmark_node],
                		target_container='/zed_multi/zed_multi_container'
            		)
            		actions.append(load_pc_node)
        	else:
            		
            		benchmark_node = Node(
                		package='zed_topic_benchmark',
                		executable='zed_topic_benchmark',
                		name='benchmark_pointcloud' + str(i),
                		namespace='zed_multi',
                		output='screen',
                		parameters=[{
                    		'topic_name': topic_name,
                    		'use_ros_log': True,
                    		'results_file_path': results_file_path,
                    		'avg_win_size': 5000
                		}]
            		)
            		actions.append(benchmark_node)
            		
    # POSITIONAL TRACKING -- Start a benchmark node for each odometry topic of each camera (only if use_positonal_tracking is True)
    if use_positional_tracking:
    	name_array = parse_array_param(names.perform(context))
    	for i in range(cam_count):

        	# Topic name to subscribe to
        	topic_name = '/zed_multi/' + name_array[i] + '/odom'

        	if( use_ipc_val=='True'):
            		
            		benchmark_node = ComposableNode(
                		package='zed_benchmark_components',
                		plugin='stereolabs::TopicBenchmarkComponent',
                		name='benchmark_positional_tracking' + str(i),
                		namespace='zed_multi',
                		parameters=[{
                    		'topic_name': topic_name,
                    		'use_ros_log': True,
                    		'results_file_path': results_file_path,
                    		'avg_win_size': 5000
                		}],
                		extra_arguments=[{'use_intra_process_comms': True}]
            		)

            		
            		load_positional_tracking_node = LoadComposableNodes(
                		composable_node_descriptions=[benchmark_node],
                		target_container='/zed_multi/zed_multi_container'
            		)
            		actions.append(load_positional_tracking_node)
        	else:
            		# Launch each benchmark in a separate process
            		benchmark_node = Node(
                		package='zed_topic_benchmark',
                		executable='zed_topic_benchmark',
                		name='benchmark_positional_tracking' + str(i),
                		namespace='zed_multi',
                		output='screen',
                		parameters=[{
                    		'topic_name': topic_name,
                    		'use_ros_log': True,
                    		'results_file_path': results_file_path,
                    		'avg_win_size': 5000
                		}]
            		)
            		actions.append(benchmark_node)
            		
    # OBJECT DETECTION -- Start a benchmark node for each object detection topic of each camera (only if use_od is True)
    if use_od:
    	name_array = parse_array_param(names.perform(context))
    	for i in range(cam_count):

        	# Topic name to subscribe to
        	topic_name = '/zed_multi/' + name_array[i] + '/obj_det/objects'

        	if( use_ipc_val=='True'):
            		
            		benchmark_node = ComposableNode(
                		package='zed_benchmark_components',
                		plugin='stereolabs::TopicBenchmarkComponent',
                		name='benchmark_od' + str(i),
                		namespace='zed_multi',
                		parameters=[{
                    		'topic_name': topic_name,
                    		'use_ros_log': True,
                    		'results_file_path': results_file_path,
                    		'avg_win_size': 5000
                		}],
                		extra_arguments=[{'use_intra_process_comms': True}]
            		)

            		
            		load_od_node = LoadComposableNodes(
                		composable_node_descriptions=[benchmark_node],
                		target_container='/zed_multi/zed_multi_container'
            		)
            		actions.append(load_od_node)
        	else:
            		# Launch each benchmark in a separate process
            		benchmark_node = Node(
                		package='zed_benchmark',
                		executable='zed_topic_benchmark',
                		name='benchmark_od' + str(i),
                		namespace='zed_multi',
                		output='screen',
                		parameters=[{
                    		'topic_name': topic_name,
                    		'use_ros_log': True,
                    		'results_file_path': results_file_path,
                    		'avg_win_size': 5000
                		}]
            		)
            		actions.append(benchmark_node)	


    ## If Performance test duration > 0, launch the required performance test node. It will kill the full launcher once the performance test is over.

    if duration != "0":
        load_performance_test_duration_node = Node(
            package="zed_benchmark",
            executable="performance_test_duration_node",
            name="performance_test_duration_node",
            output="screen",
            parameters=[{
            'performance_test_duration': performance_test_duration
            }],
            on_exit=launch.actions.Shutdown() 
        )
        actions.append(load_performance_test_duration_node)

    return actions

def generate_launch_description():
    return LaunchDescription(
        [
             DeclareLaunchArgument(
                'performance_test_duration',
                default_value='0',
                description='Duration in seconds of the benchmark performance test. If 0, the launcher runs until manual closure (CTRL+C).'),
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
                'ros_params_override_path',
                default_value='',
                description='Current ROS2 wrapper performance test configuration.'),
            DeclareLaunchArgument(
                'results_file_path',
                default_value='results.txt',
                description='Folder path to store the ROS2 wrapper performance test results.'),
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
