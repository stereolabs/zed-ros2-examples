# Tutorials
A series of tutorials are provided to better understand how to use the ZED node in the ROS2 environment :

- [Video subscribing](./zed_video_tutorial) : `zed_video_tutorial` - in this tutorial you will learn how to write a simple node that subscribes to messages of type `sensor_msgs/Image` to retrieve the Left and Right rectified images published by the ZED node.
- [Depth subscribing](./zed_depth_tutorial) : `zed_depth_tutorial` - in this tutorial you will learn how to write a simple node that subscribes to messages of type `sensor_msgs/Image` to retrieve the depth images published by the ZED node and to get the measured distance at the center of the image.
- [Pose/Odometry subscribing](./zed_pose_tutorial) : `zed_pose_tutorial` - in this tutorial you will learn how to write a simple node that subscribes to messages of type `geometry_msgs/PoseStamped` and `nav_msgs/Odometry` to retrieve the position and the odometry of the camera while moving in the world.
- [ROS2 Composition + BGRA2BGR conversion](./zed_rgb_convert) : `zed_rgb_convert` - in this tutorial you will learn how to use the concept of "ROS2 Composition" and "Intra Process Communication" to write a ROS2 component that gets a 4 channel BGRA image as input and re-publishes it as 3 channels BGR image.

For a complete explanation of the tutorials please refer to the Stereolabs ZED online documentation:

- [Video](https://www.stereolabs.com/docs/ros2/video/)
- [Depth](https://www.stereolabs.com/docs/ros2/depth_sensing/)
- [Pose/Odometry](https://www.stereolabs.com/docs/ros2/position/)
- [Composition](https://www.stereolabs.com/docs/ros2/ros2_composition/)

