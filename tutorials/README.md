# Tutorials

A series of tutorials are provided to better understand how to use the ZED nodes in the ROS 2 environment :

- [Video subscribing](./zed_video_tutorial) : `zed_video_tutorial` - in this tutorial you will learn how to write a simple node that subscribes to messages of type `sensor_msgs/Image` to retrieve the Left and Right rectified images published by the ZED node.
- [Depth subscribing](./zed_depth_tutorial) : `zed_depth_tutorial` - in this tutorial you will learn how to write a simple node that subscribes to messages of type `sensor_msgs/Image` to retrieve the depth images published by the ZED node and to get the measured distance at the center of the image.
- [Pose/Odometry subscribing](./zed_pose_tutorial) : `zed_pose_tutorial` - in this tutorial you will learn how to write a simple node that subscribes to messages of type `geometry_msgs/PoseStamped` and `nav_msgs/Odometry` to retrieve the position and the odometry of the camera while moving in the world.
- [ROS 2 Composition + BGRA2BGR conversion](./zed_rgb_convert) : `zed_rgb_convert` - in this tutorial you will learn how to use the concept of "ROS 2 Composition" and "Intra Process Communication" to write a ROS 2 component that gets a 4 channel BGRA image as input and re-publishes it as 3 channels BGR image.
- [ROS 2 Multi-Camera](./zed_multi_camera) : `zed_multi_camera` - in this tutorial you will learn how to use the provided launch file to start a multi-camera robot configuration.
- [ROS 2 Multi-Camera + Intra Process Communication](./zed_ipc) : `zed_ipc - in this tutorial you will learn how to use the provided launch file to start a multi-camera configuration, and load a new processing node in the same process to leverage Intra Process Communication with ROS 2composition.
- [Robot integration](./zed_robot_integration): `zed_robot_integration` - in this tutorial you will learn how to add one ore more ZED cameras to a robot configuration.

For a complete explanation of the tutorials please refer to the Stereolabs ZED online documentation:

- [Video](https://www.stereolabs.com/docs/ros2/video/)
- [Depth](https://www.stereolabs.com/docs/ros2/depth_sensing/)
- [Pose/Odometry](https://www.stereolabs.com/docs/ros2/position/)
- [Composition + BGRA to BGR conversion](https://www.stereolabs.com/docs/ros2/ros2_composition/)
- [Robot integration](https://www.stereolabs.com/docs/ros2/ros2_zed_integration/)
