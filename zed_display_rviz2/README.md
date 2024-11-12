# Stereolabs ZED Camera - ROS2 Display package

This package lets you visualize in
the [ROS2 RViz application](https://github.com/ros2/rviz/tree/foxy) all the
possible information that can be acquired using a Stereolabs camera.

**Note:** The main package [zed-ros2-wrapper](https://github.com/flyps/zed-ros2-wrapper)
is required to correctly execute the ROS node to acquire data from a Stereolabs 3D camera.

## Getting started

- First, be sure to have installed the main ROS package to integrate the ZED cameras in the ROS
  framework: [zed-ros2-wrapper](https://github.com/flyps/zed-ros2-wrapper)
- [Install](#Installation) the package
- Read the online documentation for [More information](https://www.stereolabs.com/docs/ros2/)

### Prerequisites

- Ubuntu 20.04
- [ZED SDK](https://www.stereolabs.com/developers/release/latest/) v3.5 or later
- [ROS2 ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html)

### Installation

The *zed_display_rviz2* is a colcon package. It has beed added to the `jablka_manipulation`
docker image. If you want to install it manually, follow the instructions below:

- Install the [zed-ros2-wrapper](https://github.com/flyps/zed-ros2-wrapper) package following
  the [installation guide](https://github.com/flyps/jablka_openmm?tab=readme-ov-file#setup)

- Install the [zed-ros2-examples](https://github.com/flyps/zed-ros2-examples) package following
  the [installation guide](https://github.com/flyps/zed-ros2-examples)

The following fork does not start the ZED camera node, but only the RVIZ2 node.

