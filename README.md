<h1 align="center">
   <img src="./images/Picto+STEREOLABS_Black.jpg" alt="Stereolabs" title="Stereolabs" /><br \>
   ROS 2 Tutorials and Examples
</h1>

This package contains examples and tutorials for effectively using ZED cameras within the ROS 2 framework.

[More information](https://www.stereolabs.com/docs/ros2/)

## Getting started

- First, be sure to have installed the main ROS 2 package to integrate the ZED cameras in the ROS 2 framework: [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper/)
- [Install](#build-the-package) the Tutorials package
- Read the online documentation for [More information](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html)

### Prerequisites

- ROS 2 Foxy Fitzroy (deprecated), ROS 2 Humble Hawksbill, or ROS2 Jazzy Jalisco:
  - [Foxy on Ubuntu 20.04](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html) - [**Not recommended. EOL reached**]
  - [Humble on Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Linux-Install-Debians.html) - [EOL May 2027]
  - [Jazzy Jalisco on Ubuntu 24.04](https://docs.ros.org/en/jazzy/Installation/Linux-Install-Debians.html) - [EOL May 2029]

### Build the package

The `zed-ros-examples` repository is a collection of [colcon](http://design.ros2.org/articles/build_tool.html) packages. 

Open a terminal, clone the repository, update the dependencies and build the packages:

```bash
cd ~/ros2_ws/src/ #use your current ros2 workspace folder
git clone https://github.com/stereolabs/zed-ros2-examples.git
cd ../
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source ~/.bashrc
```

## Tutorials and examples

### Rviz visualization examples

- Example launch files to start a preconfigured instance of Rviz displaying all the ZED Wrapper node information: [zed_display_rviz2](./zed_display_rviz2)
- ROS 2 plugin for ZED2 to visualize the results of the Object Detection module (bounding boxes and skeletons): [rviz-plugin-zed-od](./rviz-plugin-zed-od)
- Example layouts to display ZED Wrapper data in Foxglove Studio [zed_display_foxglove](./zed_display_foxglove/)

### Tutorials

- [Images subscription tutorial](./tutorials/zed_video_tutorial)
- [Depth subscription tutorial](./tutorials/zed_depth_tutorial)
- [Pose/Odometry subscription tutorial](./tutorials/zed_pose_tutorial)
- [ROS 2 Composition + BGRA2BGR conversion tutorial](./tutorials/zed_rgb_convert)
- [Multi-camera](./tutorials/zed_multi_camera)
- [Multi-Camera + Intra Process Communication](./tutorials/zed_ipc)
- [Robot integration](./tutorials/zed_robot_integration)

### Examples

- [ZED Localization with ArUco markers](./examples/zed_aruco_localization): use localized ArUco tags as a reference for localization.
- [Convert ZED Depth map to virtual laser scan](./examples/zed_depth_to_laserscan): convert ZED Depth maps into virtual Laser Scans using

 ## Tools

- **ZED Benchmark tool**: used to test topics and get statistics on frequency and bandwidth to be plotted.

 




