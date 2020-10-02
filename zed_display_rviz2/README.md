# Stereolabs ZED Camera - ROS2 Display package

This package lets you visualize in the [ROS2 RViz application](https://github.com/ros2/rviz/tree/eloquent) all the
possible information that can be acquired using a Stereolabs camera.
The package provides the launch files for ZED, ZED Mini and ZED 2 camera models.

**Note:** The main package [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
is required to correctly execute the ROS node to acquire data from a Stereolabs 3D camera.

## Getting started

   - First, be sure to have installed the main ROS package to integrate the ZED cameras in the ROS framework: [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper/#build-the-package)
   - [Install](#Installation) the package
   - Read the online documentation for [More information](https://www.stereolabs.com/docs/ros2/)

### Prerequisites

   - Ubuntu 18.04 or newer (Ubuntu 18 recommended)
   - [ZED SDK **≥ 3.0**](https://www.stereolabs.com/developers/) and its dependency [CUDA](https://developer.nvidia.com/cuda-downloads)
   - [ROS2 Eloquent Elusor](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Development-Setup/)

### Installation

The *zed_display_rviz* is a colcon package. 

Install the [zed-ros2-wrapper](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html) package
following the [installation guide](https://github.com/stereolabs/zed-ros2-wrapper#build-the-package)

Install the [zed-ros2-examples](https://github.com/stereolabs/zed-ros2-examples) package following the [installation guide](https://github.com/stereolabs/zed-ros2-examples#build-the-package)

### Execution

If you own a ZED camera launch:

    $ ros2 launch zed_display display_zed.launch.py

If you own a ZED Mini camera launch:

    $ ros2 launch zed_display display_zedm.launch.py

If you own a ZED 2 camera launch:

    $ ros2 launch zed_display display_zed2.launch.py

![ZED rendering on Rviz](images/depthcloud-RGB.jpg)
![ZED rendering on Rviz](images/ZEDM-Rviz.jpg)
![ZED rendering on Rviz](images/ZED-Rviz.jpg)

[Detailed information](https://www.stereolabs.com/docs/ros/rviz/)
