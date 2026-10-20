# ZED + Isaac ROS Object Detection Example

This example detects objects in the images of a ZED camera with the [Isaac ROS 5 object detection](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_object_detection/index.html) packages, accelerated with TensorRT. Two models are supported:

- **RT-DETR** (default): the NVIDIA [SyntheticaDETR](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/models/synthetica_detr) `sdetr_grasp` model, trained on synthetic data to detect objects that a robot arm can grasp.
- **YOLOv8**: an [Ultralytics YOLOv8](https://docs.ultralytics.com/models/yolov8/) model, by default trained on the 80 classes of the COCO dataset (people, vehicles, animals, everyday objects).

The example is designed for the monocular **ZED X One** cameras, which do not provide depth and so cannot use the Object Detection module of the ZED SDK. It works with all the ZED cameras: with stereo cameras, the objects are detected in the left image.

The images stay in GPU memory from the ZED node to the TensorRT inference, thanks to the CUDA zero-copy transport of ROS 2 Lyrical (`rosidl::Buffer` with the `cuda` buffer backend).

```text
ZED node ─► Resize ─► Pad ─► BGR to RGB ─► Image to tensor ─► Interleaved to planar ─► Reshape
 (bgr8)    (keep aspect   (640x640,                                                        │
            ratio)         bottom-right)            ┌──────────────────────────────────────┘
                                                    ▼
             [RT-DETR only] RT-DETR preprocessor ─► TensorRT ─► RT-DETR / YOLOv8 decoder
                                                                        │ detections (640x640 network image)
                                                                        ▼
                                                              Detection rescaler ─► detections, annotations
                                                                        │           (ZED image coordinates)
                                                                        ▼
                                                              NVIDIA visualizer ─► processed_image
```

All the C++ nodes run in the same component container as the ZED node.

## Requirements

- A Jetson Thor or Orin device with JetPack 7.2, or an x86_64 machine with an NVIDIA GPU (Ampere or newer), Ubuntu 24.04 and CUDA 13.2
- NVIDIA Isaac ROS 5 with ROS 2 Lyrical, in Docker or Bare Metal mode, using the `rmw_fastrtps_cpp` (default) or the `rmw_zenoh_cpp` (v0.10.5 or newer) middleware
- ZED SDK v5.1 or newer
- ZED ROS 2 Wrapper v5.6.0 or newer, built with the `cuda_buffer` package
- The `cuda_buffer_backend` plugin, built from source

Follow these guides, in Docker or in Bare Metal mode:

1. [Setting up Isaac ROS 5](https://docs.stereolabs.com/docs/integrations/isaac-ros/setup-and-installation/), to install Isaac ROS 5.
2. [Use ZED with Isaac ROS](https://docs.stereolabs.com/docs/integrations/isaac-ros/use-zed-with-isaac-ros/), to install the ZED SDK and the ZED ROS 2 Wrapper.
3. [Set up the CUDA zero-copy transport](https://docs.stereolabs.com/docs/integrations/isaac-ros/cuda-zero-copy/#set-up-the-cuda-zero-copy-transport), to install the `cuda_buffer` package, build the `cuda_buffer_backend` plugin and rebuild the ZED ROS 2 Wrapper.

> [!WARNING]
> Do not install the `ros-lyrical-cuda-buffer-backend` Debian package: it is built for the NVIDIA desktop drivers and the CUDA 12 runtime. On Jetson, it tries to install desktop driver packages that conflict with JetPack and leaves the package manager in a broken state.

Without the `cuda_buffer_backend` plugin, the example works, but the ZED node publishes the images in CPU memory and the Isaac ROS nodes upload them to the GPU. The ZED node reports the transport in use at startup, for example `* Buffer transport: CUDA zero-copy`.

## Installation

The installation steps are the same in Docker and in Bare Metal mode, with the differences described in each step. Run all the commands in the Isaac ROS environment:

- **Docker**: run `isaac-ros activate` to open a shell in the Isaac ROS container. The workspace `${ISAAC_ROS_WS}` of the host is mounted in the container, so the source code, the build artifacts and the models persist across container restarts.
- **Bare Metal**: run `isaac-ros activate` in your terminal.

### 1. Get the source code

Clone the ZED ROS 2 Examples repository in the workspace, next to the ZED ROS 2 Wrapper:

```bash
cd ${ISAAC_ROS_WS}/src
git clone https://github.com/stereolabs/zed-ros2-examples.git
```

### 2. Install the dependencies

Install the Isaac ROS packages used by the example, the tool to download the RT-DETR model, and the optional Foxglove bridge:

```bash
cd ${ISAAC_ROS_WS}
sudo apt-get update
rosdep update
rosdep install --from-paths src/zed-ros2-examples/isaac_ros/isaac_ros_5/zed_isaac_ros_object_detection --ignore-src -r -y
sudo apt-get install -y ros-lyrical-isaac-ros-rtdetr-models-install ros-lyrical-foxglove-bridge
```

`rosdep` installs the Isaac ROS image processing, tensor processing, TensorRT, RT-DETR and YOLOv8 packages, `vision_msgs` and `foxglove_msgs`.

**Docker**: the packages installed with `apt-get` in the container are lost when the container is removed, for example after rebuilding the image with `isaac-ros activate --build-local`, and you must repeat this step. To keep them, add them to a custom image layer, as you did for the ZED SDK:

1. On the host, create the `${ISAAC_ROS_WS}/docker/Dockerfile.zed_object_detection` file:

   ```dockerfile
   ARG BASE_IMAGE
   FROM ${BASE_IMAGE}

   RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
       apt-get update && apt-get install -y --no-install-recommends \
           ros-${ROS_DISTRO}-isaac-ros-image-proc \
           ros-${ROS_DISTRO}-isaac-ros-tensor-proc \
           ros-${ROS_DISTRO}-isaac-ros-tensor-rt \
           ros-${ROS_DISTRO}-isaac-ros-rtdetr \
           ros-${ROS_DISTRO}-isaac-ros-rtdetr-models-install \
           ros-${ROS_DISTRO}-isaac-ros-yolov8 \
           ros-${ROS_DISTRO}-vision-msgs \
           ros-${ROS_DISTRO}-foxglove-msgs \
           ros-${ROS_DISTRO}-foxglove-bridge \
       && rm -rf /var/lib/apt/lists/*
   ```

2. Add the `zed_object_detection` image key after `zed_sdk` in the `${ISAAC_ROS_WS}/.isaac-ros-cli/config.yaml` file:

   ```yaml
   docker:
     image:
       additional_image_keys:
         - zed_sdk
         - zed_object_detection
   ```

3. Exit all the shells of the running container, then rebuild the image and start the container with `isaac-ros activate --build-local`.

### 3. Install the models

The models are stored in the `${ISAAC_ROS_WS}/isaac_ros_assets/models` folder, which persists across container restarts in Docker mode.

The TensorRT engine file (`.plan`) is optimized for the GPU and the TensorRT version of the device where it is generated: generate it on the device that runs the example, and generate it again after a JetPack or TensorRT upgrade.

#### RT-DETR

Download the SyntheticaDETR `sdetr_grasp` model from NVIDIA NGC and generate its TensorRT engine. The script asks you to accept the model license (EULA):

```bash
ros2 run isaac_ros_rtdetr_models_install install_rtdetr_models.sh --eula
```

The model is saved in `${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/`. The engine generation takes about 2 minutes on a Jetson AGX Orin.

#### YOLOv8 (optional)

Export a YOLOv8 model to ONNX with the Ultralytics Python package. The following commands export the `yolov8s` model, trained on the COCO dataset, in a dedicated Python virtual environment:

```bash
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8
cd ${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8
sudo apt-get install -y python3-venv libgl1
python3 -m venv /tmp/yolov8_export
/tmp/yolov8_export/bin/pip install "numpy<2" ultralytics onnx onnxslim
/tmp/yolov8_export/bin/python -c "from ultralytics import YOLO; YOLO('yolov8s.pt').export(format='onnx')"
rm -rf /tmp/yolov8_export
```

The virtual environment keeps the Ultralytics packages and their dependencies (PyTorch, OpenCV) out of the system Python, which is important in Bare Metal mode: they could replace the versions required by Isaac ROS. In Docker mode, the virtual environment is removed with the container anyway.

The TensorRT engine `yolov8s.plan` is generated automatically from `yolov8s.onnx` the first time you launch the example with `model:=yolov8`, which takes about 2 minutes on a Jetson AGX Orin: the detections are published when the TensorRT node logs `TensorRT Node initialized successfully`. You can also generate it in advance:

```bash
/usr/src/tensorrt/bin/trtexec --onnx=yolov8s.onnx --saveEngine=yolov8s.plan
```

> [!NOTE]
> The Ultralytics YOLOv8 models are released under the [AGPL-3.0 license](https://www.ultralytics.com/license). Check that it is compatible with your application, or use the RT-DETR model.

### 4. Build the example

```bash
cd ${ISAAC_ROS_WS}
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to zed_isaac_ros_object_detection
source install/local_setup.bash
```

Run `source ${ISAAC_ROS_WS}/install/local_setup.bash` in every new shell. In Docker mode, run `isaac-ros activate` first in every new terminal to open a shell in the running container.

## Usage

Start the example with the RT-DETR model. Replace `<camera_model>` with the model of your camera, for example `zedxonegs`, `zedxone4k`, `zedxm` or `zed2i`:

```bash
ros2 launch zed_isaac_ros_object_detection zed_isaac_ros_object_detection.launch.py camera_model:=<camera_model>
```

Start the example with the YOLOv8 model:

```bash
ros2 launch zed_isaac_ros_object_detection zed_isaac_ros_object_detection.launch.py camera_model:=<camera_model> model:=yolov8
```

The command is the same in Docker and in Bare Metal mode.

> [!TIP]
> The `sdetr_grasp` RT-DETR model only detects the [objects listed in its model card](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/models/synthetica_detr), and its default confidence threshold is high (`0.9`). If no object is detected, place one of these objects in front of the camera, or lower the threshold, for example with `confidence_threshold:=0.6`. To test the example with common objects, such as people, use the YOLOv8 model.

### Launch arguments

| Argument | Default | Description |
| --- | --- | --- |
| `camera_model` | (required) | The model of the camera. |
| `model` | `rtdetr` | The object detection model: `rtdetr` or `yolov8`. |
| `model_file_path` | see description | The absolute path to the ONNX model file. Default: `${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.onnx` for `rtdetr`, `${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.onnx` for `yolov8`. |
| `engine_file_path` | see description | The absolute path to the TensorRT engine file, generated from the ONNX model file if it does not exist. Default: the path of the ONNX model file with the `.plan` extension. |
| `force_engine_update` | `false` | Regenerate the TensorRT engine file, even if it exists. |
| `confidence_threshold` | `0.9` (`rtdetr`), `0.25` (`yolov8`) | The minimum confidence of the published detections. |
| `nms_threshold` | `0.45` | [YOLOv8 only] The IoU threshold of the Non-Maximum Suppression. |
| `num_classes` | `80` | [YOLOv8 only] The number of classes of the model. |
| `visualizer` | `true` | Start the NVIDIA visualizer. |
| `foxglove` | `false` | Start the Foxglove bridge. |
| `foxglove_port` | `8765` | The port of the Foxglove bridge. |
| `svo_path` | `live` | Path to an SVO file, to run the example without a camera. |

To use your own YOLOv8 model, set `model_file_path` and `engine_file_path`, and `num_classes` if the model is not trained on the COCO dataset. The class names displayed by the visualizers are the COCO ones.

The ZED camera parameters are set in the [config/zed_params.yaml](config/zed_params.yaml) file. In particular, `video.enable_24bit_output: true` makes the ZED node publish `bgr8` images, the format expected by the Isaac ROS image processing nodes, and `general.pub_downscale_factor: 2.0` reduces the size of the published images, which are resized to 640x640 before the inference.

### Topics

All the topics of the example are in the `/zed_isaac/object_detection` namespace:

| Topic | Type | Description |
| --- | --- | --- |
| `detections` | `vision_msgs/Detection2DArray` | The detected objects, in the coordinates of the ZED image `/zed_isaac/zed/rgb/color/rect/image`. Use this topic in your application. |
| `detections_output` | `vision_msgs/Detection2DArray` | The detected objects as published by the Isaac ROS decoder, in the coordinates of the 640x640 network input image `padded_image`. |
| `annotations` | `foxglove_msgs/ImageAnnotations` | The bounding boxes and the labels, to be displayed over the ZED image in Foxglove. |
| `processed_image` | `sensor_msgs/Image` | The ZED image with the detections, drawn by the NVIDIA visualizer. |
| `padded_image` | `sensor_msgs/Image` | The ZED image resized and padded to the network input size (640x640). |

The class ID of each detection is in `results[0].hypothesis.class_id`, and its confidence in `results[0].hypothesis.score`.

## Visualization

### NVIDIA visualizer

With `visualizer:=true` (default), the NVIDIA visualizer of the selected model draws the detections over the ZED image and publishes the result on the `/zed_isaac/object_detection/processed_image` topic. Display it with `rqt_image_view`:

```bash
sudo apt-get install -y ros-lyrical-rqt-image-view
ros2 run rqt_image_view rqt_image_view /zed_isaac/object_detection/processed_image
```

The visualizer is a Python node that processes the images in CPU memory: set `visualizer:=false` to save CPU resources when you do not need it.

### Foxglove

Start the example with the Foxglove bridge:

```bash
ros2 launch zed_isaac_ros_object_detection zed_isaac_ros_object_detection.launch.py camera_model:=<camera_model> foxglove:=true
```

Then, in [Foxglove](https://foxglove.dev/):

1. Open a connection to `ws://<device_ip>:8765`, where `<device_ip>` is the IP address of the device running the example. In Docker mode, the Isaac ROS container uses the network of the host, so the address is the same.
2. Import the [foxglove/zed_isaac_ros_object_detection.json](foxglove/zed_isaac_ros_object_detection.json) layout, from the layouts menu, with the "Import from file..." button.

The layout displays the compressed ZED image with the bounding boxes and the labels of the `annotations` topic, the 640x640 network input image, and the content of the `detections` topic. The annotations are drawn by Foxglove, without processing the images in the ROS 2 graph.

## Troubleshooting

- **No detection is published**: check that all the terminals use the same supported middleware (`rmw_fastrtps_cpp` or `rmw_zenoh_cpp`), as described in the [Isaac ROS 5 setup guide](https://docs.stereolabs.com/docs/integrations/isaac-ros/setup-and-installation/#middleware-requirement). With the RT-DETR model, also read the tip in the [Usage](#usage) section.
- **The example stops with `Neither the TensorRT engine file ... nor the ONNX model file ... exist`**: install the model as described in [Install the models](#3-install-the-models), or set the `model_file_path` and `engine_file_path` launch arguments.
- **The example stops with `The ISAAC_ROS_WS environment variable is not set`**: set `ISAAC_ROS_WS` to the path of your Isaac ROS workspace, as described in the Isaac ROS 5 setup guide.
- **TensorRT reports an error while loading the engine file**: the engine was generated on another device or with another TensorRT version. Regenerate it with `force_engine_update:=true`.
- **The NVIDIA visualizer stops with `ImportError: numpy.core.multiarray failed to import`**: a NumPy 2 version installed with `pip` in your user folder (`~/.local/lib/python3.12/site-packages`) replaces the NumPy 1 version used by `cv_bridge`. Check it with `python3 -c "import numpy; print(numpy.__version__, numpy.__file__)"`, then remove the user version with `pip uninstall numpy`, or start the example with the `PYTHONNOUSERSITE=1` environment variable to ignore the user Python packages.
