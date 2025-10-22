# Stereolabs ZED Camera - ROS 2 Object Detection Plugin

This package lets you visualize in the [ROS 2 RViz application](https://github.com/ros2/rviz/tree/foxy) all the
information provided by the Object Detection module of the ZED SDK used together with a ZED2 camera.

**Note:** The main package [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
is required to correctly execute the ROS node to acquire data from a Stereolabs 3D camera.

## Getting started

   - First, be sure to have installed the main ROS package to integrate the ZED cameras in the ROS framework: [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper/#build-the-package)
   - [Install](#Installation) the package
   - Read the online documentation for [More information](https://www.stereolabs.com/docs/ros2/)

### Prerequisites

- ROS 2 Foxy Fitxroy or ROS 2 Humble Hawksbill: 
  - [Foxy on Ubuntu 20.04](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html)
  - [Humble on Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Linux-Install-Debians.html)

### Installation

The *zed_display_rviz2* is a colcon package. 

Install the [zed-ros2-wrapper](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html) package
following the [installation guide](https://github.com/stereolabs/zed-ros2-wrapper#build-the-package)

Install the [zed-ros2-examples](https://github.com/stereolabs/zed-ros2-examples) package following the [installation guide](https://github.com/stereolabs/zed-ros2-examples#build-the-package)

### Execution

    $ ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=<camera_model>

Please replace `<camera_model>` with the model of the camera that you are using: `'zedm'`, `'zed2'`, `'zed2i'`, `'zedx'`, `'zedxm'`, `'zedxhdrmini'`, `'zedxhdr'`, `'zedxhdrmax'`, `'virtual'`.

**Note**: Object Detection and Body Tracking are not available with the "ZED" Gen.1 camera.

![Object Detection parameters](images/rviz2_od_params.png)
![Bounding boxes and Skeleton visualization](images/rviz2_od_result.png)

[Detailed information](https://www.stereolabs.com/docs/ros2/object-detection/)
