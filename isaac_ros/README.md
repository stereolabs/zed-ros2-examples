# Isaac ROS Examples

This repository contains example applications that demonstrate how to use the [Isaac ROS](https://developer.nvidia.com/isaac-ros) SDK with the [ZED ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper).

## Prerequisites

- [NVIDIA Isaac ROS](https://docs.stereolabs.com/isaac-ros): Isaac ROS 5 with ROS 2 Lyrical, or Isaac ROS 3.2 with NITROS and ROS 2 Humble
- [ZED ROS2 Wrapper](https://docs.stereolabs.com/ros2)

## Available Examples

- [zed_isaac_ros_cuda_img_sub](https://github.com/stereolabs/zed-ros2-examples/tree/main/isaac_ros/zed_isaac_ros_cuda_img_sub): [Isaac ROS 5] An example that shows how to subscribe to ZED camera topics using the CUDA zero-copy transport of ROS 2 Lyrical (`rosidl::Buffer`) and displays communication benchmarks comparing the CUDA zero-copy and the standard CPU communication.
- [zed_isaac_ros_cuda_pc_sub](https://github.com/stereolabs/zed-ros2-examples/tree/main/isaac_ros/zed_isaac_ros_cuda_pc_sub): [Isaac ROS 5] An example that shows how to subscribe to the ZED point cloud using the CUDA zero-copy transport, prints the coordinates of its middle point read from GPU memory, and displays communication benchmarks comparing the CUDA zero-copy and the standard CPU communication.
- [zed_isaac_ros_nitros_sub](https://github.com/stereolabs/zed-ros2-examples/tree/main/isaac_ros/zed_isaac_ros_nitros_sub): An example that shows how to subscribe to ZED camera topics using NITROS and displays communication benchmarks comparing NITROS and standard ROS2 communication. Requires Isaac ROS 3.2 (NITROS was removed in Isaac ROS 5).
- [zed_isaac_ros_april_tag](https://github.com/stereolabs/zed-ros2-examples/tree/main/isaac_ros/zed_isaac_ros_april_tag): An example that creates an Isaac ROS graphs to detects AprilTags in the ZED camera feed using Isaac ROS nodes.
