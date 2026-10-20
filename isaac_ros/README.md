# Isaac ROS Examples

This folder contains example applications that demonstrate how to use the [NVIDIA Isaac ROS](https://developer.nvidia.com/isaac-ros) SDK with the [ZED ROS 2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper).

The examples are grouped by Isaac ROS version, because the two versions use different ROS 2 distributions and different GPU data transports:

| Folder | Isaac ROS | ROS 2 | GPU data transport |
| --- | --- | --- | --- |
| [isaac_ros_5](./isaac_ros_5/) | 5 | Lyrical | CUDA zero-copy (`rosidl::Buffer`) |
| [isaac_ros_3](./isaac_ros_3/) | 3.2 | Humble | NITROS |

Use the examples of the folder that matches your Isaac ROS installation.

## Isaac ROS 5

- [zed_isaac_ros_object_detection](./isaac_ros_5/zed_isaac_ros_object_detection/): object detection with the Isaac ROS RT-DETR and YOLOv8 models, designed for the ZED X One cameras.
- [zed_isaac_ros_cuda_img_sub](./isaac_ros_5/zed_isaac_ros_cuda_img_sub/): subscribe to the ZED images with the CUDA zero-copy transport, with communication benchmarks.
- [zed_isaac_ros_cuda_pc_sub](./isaac_ros_5/zed_isaac_ros_cuda_pc_sub/): subscribe to the ZED point cloud with the CUDA zero-copy transport, with communication benchmarks.

## Isaac ROS 3

- [zed_isaac_ros_april_tag](./isaac_ros_3/zed_isaac_ros_april_tag/): AprilTag detection with the Isaac ROS AprilTag node.
- [zed_isaac_ros_nitros_sub](./isaac_ros_3/zed_isaac_ros_nitros_sub/): subscribe to the ZED images with NITROS, with communication benchmarks.
- [zed_isaac_ros_multi_camera](./isaac_ros_3/zed_isaac_ros_multi_camera/): multi-camera configuration with several ZED nodes in the same container.
