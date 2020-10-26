# Stereolabs ZED Camera - ROS2 Object Detection Plugin

This package lets you visualize in the [ROS2 RViz application](https://github.com/ros2/rviz/tree/eloquent) all the
information provided by the Object Detection module of the ZED SDK used together with a ZED2 camera.

**Note:** The main package [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
is required to correctly execute the ROS node to acquire data from a Stereolabs 3D camera.

## Getting started

   - First, be sure to have installed the main ROS package to integrate the ZED cameras in the ROS framework: [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper/#build-the-package)
   - [Install](#Installation) the package
   - Read the online documentation for [More information](https://www.stereolabs.com/docs/ros2/)

### Prerequisites

   - Ubuntu 18.04 or newer (Ubuntu 18 recommended)
   - [ZED SDK](https://www.stereolabs.com/developers/release/latest/) v3.3 or later
   - [ROS2 Eloquent Elusor](https://index.ros.org/doc/ros2/Installation/Eloquent/)

### Installation

The *zed_display_rviz2* is a colcon package. 

Install the [zed-ros2-wrapper](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html) package
following the [installation guide](https://github.com/stereolabs/zed-ros2-wrapper#build-the-package)

Install the [zed-ros2-examples](https://github.com/stereolabs/zed-ros2-examples) package following the [installation guide](https://github.com/stereolabs/zed-ros2-examples#build-the-package)

### Execution

Available only if you own a ZED 2 camera:

    $ ros2 launch zed_display_rviz2 display_zed2.launch.py

![Object Detection parameters](images/rviz2_od_params.png)
![Bounding boxes and Skeleton visualization](images/rviz2_od_result.png)

[Detailed information](https://www.stereolabs.com/docs/ros2/object-detection/)
