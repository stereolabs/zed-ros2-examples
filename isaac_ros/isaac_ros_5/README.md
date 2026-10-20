# Isaac ROS 5 Examples

Examples for NVIDIA Isaac ROS 5 with ROS 2 Lyrical. The ZED images and point clouds are exchanged in GPU memory with the CUDA zero-copy transport of ROS 2 Lyrical (`rosidl::Buffer` with the `cuda` buffer backend), which replaces NITROS.

## Prerequisites

- NVIDIA Isaac ROS 5 with ROS 2 Lyrical, in Docker or Bare Metal mode: follow the [Setting up Isaac ROS 5](https://docs.stereolabs.com/docs/integrations/isaac-ros/setup-and-installation/) guide
- The ZED SDK and the ZED ROS 2 Wrapper v5.6.0 or newer: follow the [Use ZED with Isaac ROS](https://docs.stereolabs.com/docs/integrations/isaac-ros/use-zed-with-isaac-ros/) guide
- The CUDA zero-copy transport: follow the [Set up the CUDA zero-copy transport](https://docs.stereolabs.com/docs/integrations/isaac-ros/cuda-zero-copy/#set-up-the-cuda-zero-copy-transport) guide

## Available Examples

- [zed_isaac_ros_object_detection](./zed_isaac_ros_object_detection/): an example that detects objects in the ZED camera images with the Isaac ROS RT-DETR and YOLOv8 packages, accelerated with TensorRT. Designed for the monocular ZED X One cameras, it works with all the ZED cameras.
- [zed_isaac_ros_cuda_img_sub](./zed_isaac_ros_cuda_img_sub/): an example that shows how to subscribe to ZED camera topics using the CUDA zero-copy transport of ROS 2 Lyrical (`rosidl::Buffer`) and displays communication benchmarks comparing the CUDA zero-copy and the standard CPU communication.
- [zed_isaac_ros_cuda_pc_sub](./zed_isaac_ros_cuda_pc_sub/): an example that shows how to subscribe to the ZED point cloud using the CUDA zero-copy transport, prints the coordinates of its middle point read from GPU memory, and displays communication benchmarks comparing the CUDA zero-copy and the standard CPU communication.
