# ZED ArUco Localization

This example demonstrates how to use a set of ArUco markers placed at known positions and orientations in the environment to locate a ZED camera device.

The zed-aruco-localization node performs ArUco detection at a fixed rate by subscribing to ZED color image topics. When a tag is detected it calculates the camera pose with respect to it and calls the `set_pose` service of the ZED node to reset the camera pose in the World.
