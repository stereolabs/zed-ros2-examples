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
    LoadComposableNodes,
    Node
)
from launch_ros.descriptions import (
    ComposableNode
)

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

# RT-DETR and YOLOv8 models expect a 640x640 RGB image
NETWORK_IMAGE_SIZE = 640
NETWORK_NUM_CHANNELS = 3

# Model specific configurations
MODELS = {
    'rtdetr': {
        'model_dir': 'synthetica_detr',
        'model_name': 'sdetr_grasp',
        'confidence_threshold': 0.9,
        'scale_image': False,
        'input_tensor_names': ['images', 'orig_target_sizes'],
        'input_binding_names': ['images', 'orig_target_sizes'],
        'output_tensor_names': ['labels', 'boxes', 'scores'],
        'output_binding_names': ['labels', 'boxes', 'scores'],
        'visualizer': 'isaac_ros_rtdetr_visualizer.py',
        'visualizer_image_topic': 'image_rect',
        'processed_image_topic': 'rtdetr_processed_image',
    },
    'yolov8': {
        'model_dir': 'yolov8',
        'model_name': 'yolov8s',
        'confidence_threshold': 0.25,
        'scale_image': True,
        'input_tensor_names': ['input_tensor'],
        'input_binding_names': ['images'],
        'output_tensor_names': ['output_tensor'],
        'output_binding_names': ['output0'],
        'visualizer': 'isaac_ros_yolov8_visualizer.py',
        'visualizer_image_topic': '/yolov8_encoder/resize/image',
        'processed_image_topic': 'yolov8_processed_image',
    },
}


def get_model_file_paths(model_cfg, model_file_path, engine_file_path):
    """Return the ONNX and TensorRT engine paths, defaulting to the Isaac ROS assets folder."""
    if model_file_path and engine_file_path:
        return model_file_path, engine_file_path

    isaac_ros_ws = os.environ.get('ISAAC_ROS_WS', '')
    if not isaac_ros_ws:
        raise RuntimeError(
            'The ISAAC_ROS_WS environment variable is not set: set it to the path of your '
            'Isaac ROS workspace, or set both the `model_file_path` and `engine_file_path` '
            'launch arguments.')

    models_dir = os.path.join(
        isaac_ros_ws, 'isaac_ros_assets', 'models', model_cfg['model_dir'])
    if not model_file_path:
        model_file_path = os.path.join(models_dir, model_cfg['model_name'] + '.onnx')
    if not engine_file_path:
        engine_file_path = os.path.join(models_dir, model_cfg['model_name'] + '.plan')
    return model_file_path, engine_file_path


def launch_setup(context, *args, **kwargs):

    # Get the path to the camera configuration file
    camera_config_override_path = os.path.join(
        get_package_share_directory('zed_isaac_ros_object_detection'),
        'config',
        'zed_params.yaml'
    )

    # List of actions to be launched
    actions = []

    namespace_val = 'zed_isaac'
    camera_name_val = 'zed'
    od_namespace = namespace_val + '/object_detection'

    camera_model = LaunchConfiguration('camera_model')
    svo_path = LaunchConfiguration('svo_path')
    model_val = LaunchConfiguration('model').perform(context)
    confidence_threshold_val = LaunchConfiguration('confidence_threshold').perform(context)
    nms_threshold_val = float(LaunchConfiguration('nms_threshold').perform(context))
    num_classes_val = int(LaunchConfiguration('num_classes').perform(context))
    force_engine_update_val = LaunchConfiguration('force_engine_update').perform(context)
    visualizer_val = LaunchConfiguration('visualizer').perform(context)
    foxglove_val = LaunchConfiguration('foxglove').perform(context)

    model_cfg = MODELS[model_val]

    model_file_path, engine_file_path = get_model_file_paths(
        model_cfg,
        LaunchConfiguration('model_file_path').perform(context),
        LaunchConfiguration('engine_file_path').perform(context))

    if confidence_threshold_val:
        confidence_threshold = float(confidence_threshold_val)
    else:
        confidence_threshold = model_cfg['confidence_threshold']

    if not os.path.isfile(engine_file_path) and not os.path.isfile(model_file_path):
        raise RuntimeError(
            'Neither the TensorRT engine file `' + engine_file_path +
            '` nor the ONNX model file `' + model_file_path + '` exist. ' +
            'Follow the README of the example to install the `' + model_val + '` model.')

    if not os.path.isfile(engine_file_path):
        info = '* The TensorRT engine file `' + engine_file_path + \
            '` does not exist and will be generated from the ONNX model file. ' + \
            'This can take several minutes...'
        actions.append(LogInfo(msg=TextSubstitution(text=info)))

    info = '* Object detection model: `' + model_val + '` - TensorRT engine: `' + \
        engine_file_path + '`'
    actions.append(LogInfo(msg=TextSubstitution(text=info)))

    # ZED topics: the rectified color image, the left image for stereo cameras
    zed_topic_root = '/' + namespace_val + '/' + camera_name_val + '/'
    zed_image_topic = zed_topic_root + 'rgb/color/rect/image'
    zed_camera_info_topic = zed_topic_root + 'rgb/color/rect/camera_info'

    # ROS 2 Component Containers
    # The ZED node and the Isaac ROS nodes run in two different containers: the
    # ZED images are shared with the Isaac ROS nodes through cross-process CUDA IPC.
    # The same-process CUDA IPC path of `cuda_buffer_backend` can deliver an image
    # block already recycled by the ZED node, which stops the container.
    container_name = 'zed_container'
    od_container_name = 'object_detection_container'

    # Note: It is crucial that the containers use the multi-threaded executor
    #  so that the created nodes can be started and communicated correctly within the same process.
    for name, ns in [(container_name, namespace_val), (od_container_name, od_namespace)]:
        info = '* Starting Composable node container: ' + ns + '/' + name
        actions.append(LogInfo(msg=TextSubstitution(text=info)))
        actions.append(ComposableNodeContainer(
            name=name,
            namespace=ns,
            package='rclcpp_components',
            executable='component_container',
            # The buffer typesupport logs several INFO lines for each CUDA buffer message
            arguments=['--executor-type', 'multi-threaded',
                       '--ros-args', '--log-level', 'info',
                       '--log-level', 'serialize_buffer_with_endpoint:=warn',
                       '--log-level', 'deserialize_buffer_with_endpoint:=warn'],
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
            'camera_name': camera_name_val,
            'container_name': container_name,
            'namespace': namespace_val,
            'svo_path': svo_path,
            'enable_ipc': 'false',
            'ros_params_override_path': camera_config_override_path
        }.items()
    )
    actions.append(zed_wrapper_launch)

    # ----> Image encoding: ZED image to the 640x640 RGB tensor expected by the network
    # The image is resized keeping its aspect ratio, then padded on the bottom and on the
    # right side, as in the NVIDIA RT-DETR pipeline. The detections are mapped back to
    # the ZED image by the `zed_detection_rescaler` node.
    resize_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        name='resize_node',
        namespace=od_namespace,
        # `input_qos_depth: 1` (subscription queue) and `input_queue_size: 1` (image and
        # camera info synchronization queue): keep only the latest ZED image. The `cuda`
        # buffer backend can reuse the GPU memory of an image about 100 ms after it is
        # published, so an image taken late from a longer queue can no longer be read.
        parameters=[{
            'output_width': NETWORK_IMAGE_SIZE,
            'output_height': NETWORK_IMAGE_SIZE,
            'keep_aspect_ratio': True,
            'disable_padding': True,
            'input_qos_depth': 1,
            'input_queue_size': 1
        }],
        remappings=[
            ('image', zed_image_topic),
            ('camera_info', zed_camera_info_topic)
        ]
    )

    pad_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::PadNode',
        name='pad_node',
        namespace=od_namespace,
        parameters=[{
            'output_image_width': NETWORK_IMAGE_SIZE,
            'output_image_height': NETWORK_IMAGE_SIZE,
            'padding_type': 'BOTTOM_RIGHT'
        }],
        remappings=[
            ('image', 'resize/image')
        ]
    )

    # The ZED node publishes `bgr8` images (`video.enable_24bit_output: true`)
    image_format_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node',
        namespace=od_namespace,
        parameters=[{
            'encoding_desired': 'rgb8',
            'image_width': NETWORK_IMAGE_SIZE,
            'image_height': NETWORK_IMAGE_SIZE
        }],
        remappings=[
            ('image_raw', 'padded_image'),
            ('image', 'image_rgb')
        ]
    )

    # RT-DETR expects pixel values in [0, 255], YOLOv8 in [0, 1]
    image_to_tensor_node = ComposableNode(
        package='isaac_ros_tensor_proc',
        plugin='nvidia::isaac_ros::dnn_inference::ImageToTensorNode',
        name='image_to_tensor_node',
        namespace=od_namespace,
        parameters=[{
            'scale': model_cfg['scale_image'],
            'tensor_name': 'image'
        }],
        remappings=[
            ('image', 'image_rgb'),
            ('tensor', 'image_tensor')
        ]
    )

    interleaved_to_planar_node = ComposableNode(
        package='isaac_ros_tensor_proc',
        plugin='nvidia::isaac_ros::dnn_inference::InterleavedToPlanarNode',
        name='interleaved_to_planar_node',
        namespace=od_namespace,
        parameters=[{
            'input_tensor_shape': [NETWORK_IMAGE_SIZE, NETWORK_IMAGE_SIZE, NETWORK_NUM_CHANNELS]
        }],
        remappings=[
            ('interleaved_tensor', 'image_tensor')
        ]
    )

    # RT-DETR: the tensor is sent to the RT-DETR preprocessor
    # YOLOv8: the tensor is sent directly to the TensorRT node
    reshape_node = ComposableNode(
        package='isaac_ros_tensor_proc',
        plugin='nvidia::isaac_ros::dnn_inference::ReshapeNode',
        name='reshape_node',
        namespace=od_namespace,
        parameters=[{
            'output_tensor_name': 'input_tensor',
            'input_tensor_layout': 'CHW',
            'output_tensor_layout': 'NCHW',
            'input_tensor_shape': [NETWORK_NUM_CHANNELS, NETWORK_IMAGE_SIZE, NETWORK_IMAGE_SIZE],
            'output_tensor_shape': [
                1, NETWORK_NUM_CHANNELS, NETWORK_IMAGE_SIZE, NETWORK_IMAGE_SIZE]
        }],
        remappings=[
            ('tensor', 'planar_tensor'),
            ('reshaped_tensor', 'reshaped_tensor' if model_val == 'rtdetr' else 'tensor_pub')
        ]
    )
    # <---- Image encoding

    # ----> Inference
    tensor_rt_node = ComposableNode(
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        name='tensor_rt',
        namespace=od_namespace,
        parameters=[{
            'model_file_path': model_file_path,
            'engine_file_path': engine_file_path,
            'input_tensor_names': model_cfg['input_tensor_names'],
            'input_binding_names': model_cfg['input_binding_names'],
            'output_tensor_names': model_cfg['output_tensor_names'],
            'output_binding_names': model_cfg['output_binding_names'],
            'force_engine_update': force_engine_update_val.lower() == 'true',
            'verbose': False
        }]
    )
    # <---- Inference

    # ----> Model specific nodes
    if model_val == 'rtdetr':
        # The detections are generated in the network image coordinates (640x640), to be
        # rescaled to the ZED image as the YOLOv8 detections
        model_nodes = [
            ComposableNode(
                package='isaac_ros_rtdetr',
                plugin='nvidia::isaac_ros::rtdetr::RtDetrPreprocessorNode',
                name='rtdetr_preprocessor',
                namespace=od_namespace,
                parameters=[{
                    'image_width': NETWORK_IMAGE_SIZE,
                    'image_height': NETWORK_IMAGE_SIZE
                }],
                remappings=[
                    ('encoded_tensor', 'reshaped_tensor')
                ]
            ),
            ComposableNode(
                package='isaac_ros_rtdetr',
                plugin='nvidia::isaac_ros::rtdetr::RtDetrDecoderNode',
                name='rtdetr_decoder',
                namespace=od_namespace,
                parameters=[{
                    'confidence_threshold': confidence_threshold
                }]
            )
        ]
    else:
        model_nodes = [
            ComposableNode(
                package='isaac_ros_yolov8',
                plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
                name='yolov8_decoder',
                namespace=od_namespace,
                parameters=[{
                    'confidence_threshold': confidence_threshold,
                    'nms_threshold': nms_threshold_val,
                    'num_classes': num_classes_val
                }]
            )
        ]
    # <---- Model specific nodes

    # Load the Isaac ROS nodes into the object detection container
    load_od_nodes = LoadComposableNodes(
        composable_node_descriptions=[
            resize_node,
            pad_node,
            image_format_node,
            image_to_tensor_node,
            interleaved_to_planar_node,
            reshape_node,
            tensor_rt_node
        ] + model_nodes,
        target_container=od_namespace + '/' + od_container_name
    )
    actions.append(load_od_nodes)

    # Rescale the detections to the ZED image and publish the Foxglove annotations
    rescaler_params = [{
        'network_image_width': NETWORK_IMAGE_SIZE,
        'network_image_height': NETWORK_IMAGE_SIZE
    }]
    if model_val == 'yolov8':
        rescaler_params.append(os.path.join(
            get_package_share_directory('zed_isaac_ros_object_detection'),
            'config',
            'coco_classes.yaml'))
    rescaler_node = Node(
        package='zed_isaac_ros_object_detection',
        executable='zed_detection_rescaler.py',
        name='zed_detection_rescaler',
        namespace=od_namespace,
        output='screen',
        parameters=rescaler_params,
        remappings=[
            ('camera_info', zed_camera_info_topic)
        ]
    )
    actions.append(rescaler_node)

    # NVIDIA visualizer: draws the detections over the ZED image
    if visualizer_val.lower() == 'true':
        visualizer_node = Node(
            package='isaac_ros_' + model_val,
            executable=model_cfg['visualizer'],
            name=model_val + '_visualizer',
            namespace=od_namespace,
            output='screen',
            remappings=[
                ('detections_output', 'detections'),
                (model_cfg['visualizer_image_topic'], zed_image_topic),
                (model_cfg['processed_image_topic'], 'processed_image')
            ]
        )
        actions.append(visualizer_node)

    # Foxglove bridge
    # Only the ZED topics and the results of the example are bridged: the tensor topics
    # used between the Isaac ROS nodes cannot be decoded by Foxglove
    if foxglove_val.lower() == 'true':
        foxglove_bridge_node = Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': int(LaunchConfiguration('foxglove_port').perform(context)),
                'topic_whitelist': [
                    '^/' + namespace_val + '/' + camera_name_val + '/.*',
                    '^/' + od_namespace + '/(detections|annotations|processed_image|padded_image).*',
                    '^/tf(_static)?$',
                    '^/rosout$'
                ]
            }]
        )
        actions.append(foxglove_bridge_node)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'camera_model',
                description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features. With stereo cameras, the objects are detected in the left image.',
                choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'zedxnano', 'zedxhdr', 'zedxhdrmini', 'zedxhdrmax', 'virtual', 'zedxonegs', 'zedxone4k', 'zedxonehdr', 'zedxonecore']),
            DeclareLaunchArgument(
                'model',
                default_value='rtdetr',
                description='The object detection model: `rtdetr` (NVIDIA SyntheticaDETR) or `yolov8` (Ultralytics YOLOv8 trained on the COCO dataset).',
                choices=['rtdetr', 'yolov8']),
            DeclareLaunchArgument(
                'model_file_path',
                default_value='',
                description='The absolute path to the ONNX model file. Default: `${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.onnx` for `rtdetr`, `${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.onnx` for `yolov8`.'),
            DeclareLaunchArgument(
                'engine_file_path',
                default_value='',
                description='The absolute path to the TensorRT engine file. It is generated from the ONNX model file if it does not exist. Default: the path of the ONNX model file with the `.plan` extension.'),
            DeclareLaunchArgument(
                'force_engine_update',
                default_value='false',
                description='If `true`, the TensorRT engine file is regenerated from the ONNX model file, even if it exists.'),
            DeclareLaunchArgument(
                'confidence_threshold',
                default_value='',
                description='The minimum confidence of the published detections. Default: `0.9` for `rtdetr`, `0.25` for `yolov8`.'),
            DeclareLaunchArgument(
                'nms_threshold',
                default_value='0.45',
                description='[YOLOv8 only] The IoU threshold of the Non-Maximum Suppression.'),
            DeclareLaunchArgument(
                'num_classes',
                default_value='80',
                description='[YOLOv8 only] The number of classes of the model. The default is the number of classes of the COCO dataset.'),
            DeclareLaunchArgument(
                'visualizer',
                default_value='true',
                description='If `true`, start the NVIDIA visualizer, which publishes the ZED image with the detections on the `/zed_isaac/object_detection/processed_image` topic.'),
            DeclareLaunchArgument(
                'foxglove',
                default_value='false',
                description='If `true`, start the Foxglove bridge, to display the detections in Foxglove.'),
            DeclareLaunchArgument(
                'foxglove_port',
                default_value='8765',
                description='The port of the Foxglove bridge.'),
            DeclareLaunchArgument(
                'svo_path',
                default_value=TextSubstitution(text='live'),
                description='Path to an input SVO file, to run the example without a camera.'),
            OpaqueFunction(function=launch_setup)
        ]
    )
