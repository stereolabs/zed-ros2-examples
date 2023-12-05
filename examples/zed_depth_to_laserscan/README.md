# ZED Depth map to Laser Scan

This example demonstrates how to use [ROS 2 Composition](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html) to convert the ZED Depth map messages into a virtual laser scan by using the `depthimage_to_laserscan` package.

Usage:

```
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model>
```

## The launch file explained

The launch files creates a ROS 2 container to run the two components 