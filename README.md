![](./images/Picto+STEREOLABS_Black.jpg)

# Stereolabs ZED Camera - ROS2 Tutorials and Examples

This package is a collection of examples and tutorials to illustrate how to better use the ZED cameras in the ROS2 framework

[More information](https://www.stereolabs.com/docs/ros2/)

## Getting started

- First, be sure to have installed the main ROS2 package to integrate the ZED cameras in the ROS2 framework: [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper/)
- [Install](#build-the-package) the Tutorials package
- Read the online documentation for [More information](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html)

### Prerequisites

- Ubuntu 18.04
- [ZED SDK **≥ 3.0**](https://www.stereolabs.com/developers/) and its dependency [CUDA](https://developer.nvidia.com/cuda-downloads)
- [ROS2 Eloquent Elusor](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Development-Setup/)

### Build the package

The `zed-ros-examples` repository is a collection of [colcon](http://design.ros2.org/articles/build_tool.html) packages. 

Open a terminal, clone the repository, update the dependencies and build the packages:

    $ cd ~/ros2_ws/src/ #use your current ros2 workspace folder
    $ git clone https://github.com/stereolabs/zed-ros2-examples.git
    $ cd ../
    $ rosdep install --from-paths src --ignore-src -r -y
    $ colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
    $ source ~/.bashrc

## Run the tutorials and examples

### Rviz visualization examples

 - Example launch files to start a preconfigured instance of Rviz displaying all the ZED Wrapper node information: [zed_display_rviz2](https://github.com/stereolabs/zed-ros2-examples/tree/master/zed_display_rviz2)

### Tutorials

 - [Image subscription tutorial](https://github.com/stereolabs/zed-ros-examples/tree/master/tutorials/zed_video_sub_tutorial/README.md)
 - [Depth subscription tutorial](https://github.com/stereolabs/zed-ros-examples/tree/master/tutorials/zed_depth_sub_tutorial/README.md)




