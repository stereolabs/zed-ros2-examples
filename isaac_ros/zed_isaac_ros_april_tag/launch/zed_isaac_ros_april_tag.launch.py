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

    # Get the path to the camera configuration file
    camera_config_override_path = os.path.join(
        get_package_share_directory('zed_isaac_ros_april_tag'),
        'config',
        'zed_params.yaml'
    )

    # Get the path to the AprilTag configuration file
    apriltag_config_path = os.path.join(
        get_package_share_directory('zed_isaac_ros_april_tag'),
        'config',
        'zed_isaac_ros_april_tag.yaml'
    )

    # List of actions to be launched
    actions = []

    namespace_val = 'zed_isaac'
    disable_tf = LaunchConfiguration('disable_tf')
    camera_model = LaunchConfiguration('camera_model')

    disable_tf_val = disable_tf.perform(context)
    
    # ROS 2 Component Container
    container_name = 'zed_container'
    info = '* Starting Composable node container: ' + namespace_val + '/' + container_name
    actions.append(LogInfo(msg=TextSubstitution(text=info)))

    # Note: It is crucial that the 'executable' field is set to be 'component_container_mt'
    #  so that the created nodes can be started and communicated correctly within the same process.

    zed_container = ComposableNodeContainer(
        name=container_name,
        namespace=namespace_val,
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
    )
    actions.append(zed_container)

   
    # ZED Wrapper launch file
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
            'ros_params_override_path': camera_config_override_path
        }.items()
    )
    actions.append(zed_wrapper_launch)

    # Read the resolution from the ZED parameters file
    with open(camera_config_override_path, 'r') as f:
        configuration = yaml.safe_load(f)
        print(f'Loaded configuration: {configuration}')
        resolution = configuration["/**"]["ros__parameters"]["general"]["grab_resolution"]
        pub_resolution = configuration["/**"]["ros__parameters"]["general"]["pub_resolution"]
        pub_downscale_factor = configuration["/**"]["ros__parameters"]["general"]["pub_downscale_factor"]
        if pub_resolution == 'CUSTOM':
            rescale = pub_downscale_factor
        else:
            rescale = 1.0

    if resolution == 'HD2K':
        image_width = 2208
        image_height = 1242
    elif resolution == 'HD1200':
        image_width = 1280
        image_height = 720
    elif resolution == 'HD1080':
        image_width = 1920
        image_height = 1080
    elif resolution == 'HD720':
        image_width = 1280
        image_height = 720
    elif resolution == 'SVGA':
        image_width = 960
        image_height = 600
    elif resolution == 'VGA':
        image_width = 672
        image_height = 376

    # Isaac ROS Node to convert from ZED BGRA8 image to BGR8 required by AprilTag
    isaac_converter_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='zed_image_converter',
        namespace=namespace_val,
        parameters=[
            {
                'image_width': int(image_width / rescale),
                'image_height': int(image_height / rescale),
                'encoding_desired': 'bgr8',
                'num_blocks': 40
            }
        ],
        remappings=[
            ('image_raw', 'zed/rgb/image_rect_color'),
            ('image', 'zed/rgb/image_rect_color_bgr8')
        ]
    )

    # add parameters for conversion node -> https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_image_pipeline/isaac_ros_image_proc/index.html#imageformatconverternode

    # AprilTag detection node
    isac_apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace=namespace_val,
        remappings=[
                ('image', 'zed/rgb/image_rect_color_bgr8'),
                ('camera_info', 'zed/rgb/camera_info')
        ],
        parameters=[apriltag_config_path]
    )

    container_full_name = namespace_val + '/' + container_name
    # Load the Converter node into the container
    load_converter_node = LoadComposableNodes(
        composable_node_descriptions=[isaac_converter_node],
        target_container=container_full_name
    )
    actions.append(load_converter_node)

    # Load the AprilTag node into the container
    load_april_tag_node = LoadComposableNodes(
        composable_node_descriptions=[isac_apriltag_node],
        target_container=container_full_name
    )
    actions.append(load_april_tag_node)

    return actions

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'camera_model',
                description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
                choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual', 'zedxonegs', 'zedxone4k']),
            DeclareLaunchArgument(
                'disable_tf',
                default_value='False',
                description='If `True` disable TF broadcasting for all the cameras in order to fuse visual odometry information externally.'),
            OpaqueFunction(function=launch_setup)
        ]
    )
