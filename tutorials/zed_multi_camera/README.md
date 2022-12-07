# Multi camera tutorial

In this tutorial you will learn how to create a launch file to configure and start multiple ZED nodes 
for a multi-camera configuration.

## The launch file explained

The example launch file `zed_multi_camera.launch.py` allows to dynamically configure a robotics system equipped with more ZED camera of different models.

The number of ZED nodes to start is infered by the size of the parameters passed to the launch file from the command line. All the parameters are required and the arrays must have the same size:

* `cam_names`: An array containing the names of the cameras, e.g. `'[zed_front,zed_back,zed_left, zed_right]'`. Note: it is important that all the cameras have a different name to correctly distinguish them.
* `cam_models`: An array containing the names of the cameras, e.g. `'[zed2i,zed2,zed2,zed2i]'`
* `cam_serials`: An array containing the serial numbers of the cameras, e.g. `[3001234,2001234,2004321,3004321]`
* `cam_poses`: An array containing the array of the pose of the cameras with respect to the base frame link, e.g. `[[0.5,0.0,0.0,0.0,0.0,0.0],[0.0,0.2,0.0,0.0,1.571,0.0]],[0.0,-0.2,0.0,0.0,-1.571,0.0],[-0.5,0.0,0.0,0.0,0.0,3.142]]]`
* `disable_tf`: Disable TF broadcasting for all the cameras in ordet to fuse visual odometry information externally. [default: 'false']

The TF broadcasting is disabled for all the cameras, but the first which is configured to publish the `map` -> `odom` -> `base_link` TF chain.

## Example

Example launch command for a rig made with two cameras, the second is placed on top of the first looking backward:

    $ ros2 launch zed_multi_camera zed_multi_camera.launch.py cam_names:='[zed_front,zed_back]' cam_models:='[zed2i,zed2]' cam_serials:='[35199186,23154724]' cam_poses:=[[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.03,0.0,0.0,3.142]]




