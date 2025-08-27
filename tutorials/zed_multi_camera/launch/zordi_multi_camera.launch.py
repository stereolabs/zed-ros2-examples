# Copyright 2024 Stereolabs
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
    Command,
    TextSubstitution
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer
)

def parse_array_param(param):
    str = param.replace('[', '')
    str = str.replace(']', '')
    str = str.replace(' ', '')
    arr = str.split(',')

    return arr

def load_camera_config(config_path):
    """Load camera configuration from YAML file."""
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
            return config
    except Exception as e:
        print(f"Warning: Could not load config file {config_path}: {e}")
        return None

def get_camera_mapping(config, camera_name):
    """Get camera mapping from config - no defaults, explicit configuration required."""
    if config is None or 'cameras' not in config:
        raise ValueError(f"No camera configuration found for '{camera_name}'")

    if camera_name not in config['cameras']:
        raise ValueError(f"Camera '{camera_name}' not found in configuration file")

    return config['cameras'][camera_name]

def validate_camera_config(camera_config, camera_name):
    """Validate that all required fields are present in camera config."""
    required_fields = ['camera_model', 'serial_number', 'camera_id', 'sub_namespace', 'camera_name']
    missing_fields = []

    for field in required_fields:
        if field not in camera_config:
            missing_fields.append(field)

    if missing_fields:
        raise ValueError(f"Camera '{camera_name}' missing required fields: {missing_fields}")

    return True

def launch_setup(context, *args, **kwargs):

    # List of actions to be launched
    actions = []

    # Load camera configuration first
    config_file = LaunchConfiguration('config_file')
    config_file_path = config_file.perform(context)
    camera_config = load_camera_config(config_file_path)

    # Get namespace from config file
    namespace_val = camera_config.get('namespace', 'camera') if camera_config else 'camera'

    # URDF/xacro file to be loaded by the Robot State Publisher node
    multi_zed_xacro_path = os.path.join(
    get_package_share_directory('zed_multi_camera'),
    'urdf',
    'zed_multi.urdf.xacro')

    # Validate config file was loaded
    if camera_config is None:
        return [
            LogInfo(msg=TextSubstitution(
                text='ERROR: Could not load camera configuration file. Please provide a valid config_file path.'))
        ]

    # Get camera list from config
    if 'cameras' not in camera_config:
        return [
            LogInfo(msg=TextSubstitution(
                text='ERROR: No cameras defined in configuration file. Please add cameras section to your config.'))
        ]

    cameras_config = camera_config['cameras']
    names_arr = list(cameras_config.keys())
    num_cams = len(names_arr)

    if num_cams == 0:
        return [
            LogInfo(msg=TextSubstitution(
                text='ERROR: No cameras configured in the configuration file.'))
        ]

    # Get disable_tf setting from config (default to False)
    disable_tf_val = str(camera_config.get('disable_tf', False)).lower()

    # ROS 2 Component Container - Single shared container
    container_name = camera_config.get('container', {}).get('name', 'zed_multi_container')
    distro = os.environ['ROS_DISTRO']
    if distro == 'foxy':
        # Foxy does not support the isolated mode
        container_exec='component_container'
    else:
        container_exec='component_container_isolated'

    # Create single shared container in base namespace (now supported by ZED wrapper fix)
    info = '* Starting Composable node container: /' + namespace_val + '/' + container_name
    actions.append(LogInfo(msg=TextSubstitution(text=info)))

    zed_container = ComposableNodeContainer(
        name=container_name,
        namespace=namespace_val,
        package='rclcpp_components',
        executable=container_exec,
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
    )
    actions.append(zed_container)

    # Set the first camera idx
    cam_idx = 0

    for name in names_arr:
        try:
            # Get camera configuration from config file
            camera_mapping = get_camera_mapping(camera_config, name)

            # Validate camera configuration
            validate_camera_config(camera_mapping, name)

            # Use values directly from config (no fallbacks)
            model = camera_mapping['camera_model']
            serial = camera_mapping['serial_number']
            camera_id = str(camera_mapping['camera_id'])
            sub_namespace = camera_mapping['sub_namespace']  # static_side or static_center
            camera_name = camera_mapping['camera_name']      # zedxmini

            # Create hierarchical namespace for proper topic structure
            # namespace: /camera/static_side, camera_name: zedxmini
            # Results in: /camera/static_side/zedxmini/image_raw
            hierarchical_namespace = namespace_val + '/' + sub_namespace

        except ValueError as e:
            return [
                LogInfo(msg=TextSubstitution(
                    text=f'ERROR: {str(e)}. Please check your configuration file.'))
            ]

        # Use absolute container path (enabled by ZED wrapper fix)
        absolute_container_path = '/' + namespace_val + '/' + container_name

        info = '* Starting a ZED ROS2 node for camera ' + name + \
            ' (' + model + ', namespace: ' + hierarchical_namespace + ', camera_name: ' + camera_name + ', shared_container: ' + absolute_container_path
        if(serial != '0'):
            info += ', serial: ' + serial
        elif( camera_id != '-1'):
            info += ', id: ' + camera_id
        info += ')'

        actions.append(LogInfo(msg=TextSubstitution(text=info)))

        # Only the first camera send odom and map TF
        publish_tf = 'false'
        if (cam_idx == 0):
            if (disable_tf_val == 'False' or disable_tf_val == 'false'):
                publish_tf = 'true'

        # Add the node
        # ZED Wrapper launch file
        zed_wrapper_launch = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                get_package_share_directory('zed_wrapper'),
                '/launch/zed_camera.launch.py'
            ]),
            launch_arguments={
                'container_name': absolute_container_path,  # Absolute path to shared container
                'camera_name': camera_name,  # zedxmini - becomes the node name
                'camera_model': model,
                'serial_number': serial,
                'camera_id': camera_id,
                'publish_tf': publish_tf,
                'publish_map_tf': publish_tf,
                'namespace': hierarchical_namespace,  # Hierarchical namespace: /camera/static_side
                'node_name': camera_name  # Will be ignored but kept for clarity
            }.items()
        )
        actions.append(zed_wrapper_launch)

        cam_idx += 1

    # Create the Xacro command with correct camera names
    xacro_command = []
    xacro_command.append('xacro')
    xacro_command.append(' ')
    xacro_command.append(multi_zed_xacro_path)
    xacro_command.append(' ')
    cam_idx = 0
    for name in names_arr:
        try:
            # Get camera mapping from config for URDF consistency
            camera_mapping = get_camera_mapping(camera_config, name)
            validate_camera_config(camera_mapping, name)
            # Use the sub_namespace for URDF (static_side, static_center)
            urdf_camera_name = camera_mapping['sub_namespace']
        except ValueError as e:
            return [
                LogInfo(msg=TextSubstitution(
                    text=f'ERROR in URDF generation: {str(e)}. Please check your configuration file.'))
            ]

        xacro_command.append('camera_name_'+str(cam_idx)+':=')
        xacro_command.append(urdf_camera_name)
        xacro_command.append(' ')
        cam_idx+=1

    # Robot State Publisher node
    # this will publish the static reference link for a multi-camera configuration
    # and all the joints. See 'urdf/zed_dual.urdf.xacro' as an example
    rsp_name = 'state_publisher'
    info = '* Starting robot_state_publisher node to link all the frames: ' + rsp_name
    actions.append(LogInfo(msg=TextSubstitution(text=info)))
    multi_rsp_node = Node(
        package='robot_state_publisher',
        namespace=namespace_val,
        executable='robot_state_publisher',
        name=rsp_name,
        output='screen',
        parameters=[{
            'robot_description': Command(xacro_command).perform(context)
        }]
    )

    # Add the robot_state_publisher node to the list of nodes to be started
    actions.append(multi_rsp_node)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                default_value=os.path.join(
                    get_package_share_directory('zed_multi_camera'),
                    'configs',
                    'z-03.yaml'
                ),
                description='Path to the YAML configuration file containing all camera configurations, namespaces, hardware details, and settings.'),
            OpaqueFunction(function=launch_setup)
        ]
    )
