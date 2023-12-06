![](./images/Picto+STEREOLABS_Black.jpg)

# Stereolabs ZED Camera - ROS2 Tutorials and Examples

This package is a collection of examples and tutorials to illustrate how to better use the ZED cameras in the ROS2 framework

[More information](https://www.stereolabs.com/docs/ros2/)

## Getting started

- First, be sure to have installed the main ROS2 package to integrate the ZED cameras in the ROS2 framework: [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper/)
- [Install](#build-the-package) the Tutorials package
- Read the online documentation for [More information](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html)

### Prerequisites

- ROS2 Foxy Fitxroy or ROS2 Humble Hawksbill: 
  - [Foxy on Ubuntu 20.04](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html)
  - [Humble on Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Linux-Install-Debians.html)

### Build the package

The `zed-ros-examples` repository is a collection of [colcon](http://design.ros2.org/articles/build_tool.html) packages. 

Open a terminal, clone the repository, update the dependencies and build the packages:

    $ cd ~/ros2_ws/src/ #use your current ros2 workspace folder
    $ git clone https://github.com/stereolabs/zed-ros2-examples.git
    $ cd ../
    $ rosdep install --from-paths src --ignore-src -r -y
    $ colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
    $ source ~/.bashrc

## Tutorials and examples

### Rviz visualization examples

 - Example launch files to start a preconfigured instance of Rviz displaying all the ZED Wrapper node information: [zed_display_rviz2](https://github.com/stereolabs/zed-ros2-examples/tree/master/zed_display_rviz2)
 - ROS2 plugin for ZED2 to visualize the results of the Object Detection module (bounding boxes and skeletons): [rviz-plugin-zed-od](https://github.com/stereolabs/zed-ros2-examples/tree/master/rviz-plugin-zed-od)

### Tutorials

 - [Images subscription tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_video_tutorial)
 - [Depth subscription tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_depth_tutorial)
 - [Pose/Odometry subscription tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_pose_tutorial)
 - [ROS2 Composition + BGRA2BGR conversion tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_rgb_convert)
 - [Multi-camera](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_multi_camera)

### Examples

* [ZED Localization with ArUco markers](./zed_aruco_localization): use localized ArUco tags as a reference for localization.
* [Convert ZED Depth map to virtual laser scan](./zed_depth_to_laserscan): convert ZED Depth maps into virtual Laser Scans using [ROS 2 Composition](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html).

 ## Tools

* **ZED Benchmark tool**: used to test topics and get statistics on frequency and bandwidth to be plotted.

 




