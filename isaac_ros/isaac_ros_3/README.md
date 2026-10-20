# Isaac ROS 3 Examples

Examples for NVIDIA Isaac ROS 3.2 with ROS 2 Humble. The ZED images are exchanged in GPU memory with NITROS, which was removed in Isaac ROS 5: for Isaac ROS 5 and ROS 2 Lyrical, use the [Isaac ROS 5 examples](../isaac_ros_5/).

## Prerequisites

- NVIDIA Isaac ROS 3.2 with ROS 2 Humble: follow the [legacy Isaac ROS 3 installation guide](https://docs.stereolabs.com/docs/integrations/isaac-ros/legacy/setup-and-installation/)
- The ZED SDK and the ZED ROS 2 Wrapper, built with NITROS support as described in the same guide

## Available Examples

- [zed_isaac_ros_april_tag](./zed_isaac_ros_april_tag/): an example that creates an Isaac ROS graph to detect AprilTags in the ZED camera feed using Isaac ROS nodes.
- [zed_isaac_ros_nitros_sub](./zed_isaac_ros_nitros_sub/): an example that shows how to subscribe to ZED camera topics using NITROS and displays communication benchmarks comparing NITROS and standard ROS 2 communication.
- [zed_isaac_ros_multi_camera](./zed_isaac_ros_multi_camera/): a tutorial that shows how to create a multi-camera configuration with ZED devices, loading all the ZED nodes in the same container to which other components, such as Isaac ROS nodes, can be added.
