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
import launch
import launch_ros.actions
from launch.conditions import IfCondition

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
from launch_ros.descriptions import ComposableNode

# Function to parse array parameters
def parse_array_param(param):
    str = param.replace('[', '')
    str = str.replace(']', '')
    str = str.replace(' ', '')
    arr = str.split(',')
    return arr

def launch_setup(context, *args, **kwargs):

    # get package directory
    pkg_dir = get_package_share_directory('zed_ros2_wrapper_qa') # Current package directory path
    rviz_config_path = os.path.join(pkg_dir, 'rviz/default_view_ipc.rviz') #Rviz Config file path

    # List of actions to be launched
    actions = []

    # Arguments
    names = LaunchConfiguration('cam_names')
    models = LaunchConfiguration('cam_models')
    serials = LaunchConfiguration('cam_serials')
    disable_tf = LaunchConfiguration('disable_tf')
    shutdown_time = LaunchConfiguration('shutdown_time')
    results_file_path = LaunchConfiguration('results_file_path')
    test_name = LaunchConfiguration('test_name')
    use_rviz = LaunchConfiguration('use_rviz')
    use_image = LaunchConfiguration('use_image')
    use_pointcloud = LaunchConfiguration('use_pointcloud')
    use_obj = LaunchConfiguration('use_obj')
    use_ipc = LaunchConfiguration('use_ipc')
    rviz_config_path = LaunchConfiguration('rviz_config_file')
    config_path = LaunchConfiguration('ros_params_override_path')
    urdf_mode = LaunchConfiguration('urdf_mode')

    # Call the multi-camera launch file
    multi_camera_launch_file = os.path.join(
        get_package_share_directory('zed_ros2_wrapper_qa'),
        'launch',
        'zed_multi_camera.launch.py'
    )
    zed_multi_camera = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(multi_camera_launch_file),
        launch_arguments={
            'cam_names': names,
            'cam_models': models,
            'cam_serials': serials,
            'disable_tf': disable_tf,
            'shutdown_time': shutdown_time,
            'urdf_mode': urdf_mode
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
        
    for i in range(cam_count):
        base_topic = 'image_' + str(i)
        remap = '/zed_multi/' + name_array[i] + '/rgb/color/rect/image'
        remapping = (base_topic, remap)
        remappings.append(remapping)

    for i in range(cam_count):
        base_topic = 'objects_' + str(i)
        remap = '/zed_multi/' + name_array[i] + '/obj_det/objects'
        remapping = (base_topic, remap)
        remappings.append(remapping)


    # Create the point cloud node
    pc_node = ComposableNode(
        package='zed_ros2_wrapper_qa',
        plugin='stereolabs::PerformanceTest',
        name='zed_ros2_wrapper_qa',
        namespace='zed_multi',
        parameters=[{
            'cam_count': cam_count,
            'results_file_path': results_file_path,
            'test_name': test_name,
            'use_image': use_image,
            'use_pointcloud': use_pointcloud,
            'use_obj': use_obj,
            'use_ipc': use_ipc,
            'use_rviz': use_rviz,
            'ros_params_override_path':config_path
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
    
    load_performance_test_duration_node = Node(
            package="zed_ros2_wrapper_qa",
            executable="performance_test_duration_node",
            name="performance_test_duration_node",
            output="screen",
            parameters=[{
            'shutdown_time': shutdown_time
            }],
            on_exit=launch.actions.Shutdown() 
    )
    actions.append(load_performance_test_duration_node)
    
    #Start RVIZ node 
    load_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz)
    )
   
    actions.append(load_rviz_node)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'performance_test_duration',
                default_value='30',
                description='Duration in seconds of the zed wrapper performance test'),
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
                'results_file_path',
                default_value='',
                description='Folder path to store the ROS2 wrapper performance test results.'),
            DeclareLaunchArgument(
                'ros_params_override_path',
                default_value='',
                description='Current ROS2 wrapper performance test configuration.'),
            DeclareLaunchArgument(
                'rviz_config_file',
                default_value='',
                description='RVIZ config file'),
            DeclareLaunchArgument(
                'test_name',
                default_value='',
                description='current ROS wrapper QA test name.'),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='False',
                description='If `True` launch rviz with specific config file'),
            DeclareLaunchArgument(
                'urdf_mode',
                default_value='quad',
                description='URDF config mode: quad or duo'),
            DeclareLaunchArgument(
                'use_pointcloud',
                default_value='False',
                description='If `True` the QA test monitors pointcloud topics'),
            DeclareLaunchArgument(
                'use_obj',
                default_value='False',
                description='If `True` the QA test monitors object detection topics'),
             DeclareLaunchArgument(
                'use_image',
                default_value='False',
                description='If `True` the QA test monitors image topics'),
             DeclareLaunchArgument(
                'use_ipc',
                default_value='True',
                description='If `True` the QA test monitors with ipc'),
            OpaqueFunction(function=launch_setup)
        ]
    )
