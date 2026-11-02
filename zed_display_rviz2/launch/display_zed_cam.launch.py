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
import re
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    LogInfo
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
from launch_ros.actions import Node

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

# RVIZ2 Configurations folder
default_config_rviz2 = os.path.join(
    get_package_share_directory('zed_display_rviz2'),
    'rviz2',
    'zed'
)

# ZED Wrapper launch file
zed_wrapper_launch_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'launch',
    'zed_camera.launch.py'
)

# Camera name, namespace and node name used in the RVIZ2 configuration files
rviz2_config_camera_name = 'zed'
rviz2_config_namespace = 'zed'
rviz2_config_node_name = 'zed_node'


def _get_rviz2_config(config_path, namespace, node_name, camera_name):
    """Return an RVIZ2 configuration matching the ZED node topics and TF frames.

    The configuration files refer to the default `/zed/zed_node/` topics and
    `zed_*` TF frames. When different names are used, a copy of the file with
    the names replaced is generated in the temporary folder.
    """
    if (namespace == rviz2_config_namespace and
            node_name == rviz2_config_node_name and
            camera_name == rviz2_config_camera_name):
        return config_path

    with open(config_path, 'r', encoding='utf-8') as config_file:
        config = config_file.read()

    # Topics published by the ZED node
    config = config.replace(
        '/' + rviz2_config_namespace + '/' + rviz2_config_node_name + '/',
        '/' + namespace + '/' + node_name + '/')
    # Robot description published by the Robot State Publisher node
    config = config.replace(
        '/' + rviz2_config_namespace + '/' +
        rviz2_config_camera_name + '_description',
        '/' + namespace + '/' + camera_name + '_description')
    # TF frames
    config = re.sub(
        r'\b' + rviz2_config_camera_name + r'_(?=(camera|left|right)_)',
        camera_name + '_',
        config)

    custom_config_path = os.path.join(
        tempfile.gettempdir(),
        'zed_display_' + namespace.replace('/', '_') + '_' + node_name +
        '_' + os.path.basename(config_path))
    with open(custom_config_path, 'w', encoding='utf-8') as config_file:
        config_file.write(config)

    return custom_config_path


def launch_setup(context, *args, **kwargs):
    return_array = []

    # Launch configuration variables
    node_log_type = LaunchConfiguration('node_log_type')

    start_zed_node = LaunchConfiguration('start_zed_node')

    namespace = LaunchConfiguration('namespace')
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')

    node_name = LaunchConfiguration('node_name')

    publish_svo_clock = LaunchConfiguration('publish_svo_clock')
    use_sim_time = LaunchConfiguration('use_sim_time')

    node_log_type_val = node_log_type.perform(context)
    namespace_val = namespace.perform(context)
    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    node_name_val = node_name.perform(context)
    publish_svo_clock_val = publish_svo_clock.perform(context)
    use_sim_time_val = use_sim_time.perform(context)

    stereo_models = (
        'zed', 'zedm', 'zed2', 'zed2i',
        'zedx', 'zedxm', 'zedxnano',
        'zedxhdr', 'zedxhdrmini', 'zedxhdrmax',
        'virtual'
    )
    is_stereo_model = camera_model_val in stereo_models

    if (node_log_type_val == 'both'):
        node_log_effective = 'both'
    else:  # 'screen' or 'log'
        node_log_effective = {
            'stdout': node_log_type_val,
            'stderr': node_log_type_val
        }

    if (camera_name_val == ''):
        camera_name_val = 'zed'

    # Same naming rules as the ZED Wrapper launch file
    zed_node_name_val = node_name_val
    if (namespace_val == ''):
        namespace_val = camera_name_val
    else:
        zed_node_name_val = camera_name_val

    # RVIZ2 configuration file
    if is_stereo_model:
        config_rviz2_path_val = default_config_rviz2 + '_stereo.rviz'
    else:
        config_rviz2_path_val = default_config_rviz2 + '_mono.rviz'

    config_rviz2_path_val = _get_rviz2_config(
        config_rviz2_path_val, namespace_val, zed_node_name_val,
        camera_name_val)

    info = 'Using RVIZ2 configuration file: ' + config_rviz2_path_val
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # RVIZ2 must use the same clock as the ZED node
    rviz2_use_sim_time = (publish_svo_clock_val.lower() == 'true' or
                          use_sim_time_val.lower() == 'true')

    # RVIZ2 node
    rviz2_node = Node(
        package='rviz2',
        namespace=namespace_val,
        executable='rviz2',
        name=camera_name_val + '_rviz2',
        output=node_log_effective,
        arguments=['-d', config_rviz2_path_val],
        parameters=[{'use_sim_time': rviz2_use_sim_time}]
    )
    return_array.append(rviz2_node)

    # ZED Wrapper launch file
    # Note: all the other ZED Wrapper launch arguments (e.g. `serial_number`,
    # `svo_path`, `ros_params_override_path`, ...) are forwarded automatically
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            zed_wrapper_launch_path),
        launch_arguments={
            'node_log_type': node_log_type_val,
            'namespace': namespace.perform(context),
            'camera_name': camera_name_val,
            'camera_model': camera_model_val,
            'node_name': node_name_val,
            'publish_svo_clock': publish_svo_clock_val,
            'use_sim_time': use_sim_time_val
        }.items(),
        condition=IfCondition(start_zed_node)
    )
    return_array.append(zed_wrapper_launch)

    return return_array


def generate_launch_description():
    return LaunchDescription(
        [
            # Declare launch arguments
            DeclareLaunchArgument(
                'node_log_type',
                default_value=TextSubstitution(text='both'),
                description='The log type of the nodes. It can be `screen`, `log` or `both`. The `log` type will save the log in a file in the `~/.ros/log/` folder. The `screen` type will print the log on the terminal. The `both` type will do both.',
                choices=['screen', 'log', 'both']),
            DeclareLaunchArgument(
                'start_zed_node',
                default_value='true',
                description='Set to `false` to start only RVIZ2 if a ZED node is already running.'),
            DeclareLaunchArgument(
                'camera_name',
                default_value=TextSubstitution(text='zed'),
                description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.'),
            DeclareLaunchArgument(
                'camera_model',
                description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
                choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'zedxnano', 'zedxhdr', 'zedxhdrmini', 'zedxhdrmax', 'virtual', 'zedxonegs', 'zedxone4k', 'zedxonehdr', 'zedxonecore']),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='The namespace of the node. If empty (default) the camera name is used.'),
            DeclareLaunchArgument(
                'node_name',
                default_value='zed_node',
                description='The name of the zed_wrapper node. All the topic will have the same prefix: `/<camera_name>/<node_name>/`. If a namespace is specified, the node name is replaced by the camera name.'),
            DeclareLaunchArgument(
                'publish_svo_clock',
                default_value='false',
                description='If set to `true` the node will act as a clock server publishing the SVO timestamp. This is useful for node synchronization'),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='If set to `true` the node will wait for messages on the `/clock` topic to start and will use this information as the timestamp reference',
                choices=['true', 'false']),
            OpaqueFunction(function=launch_setup)
        ]
    )
