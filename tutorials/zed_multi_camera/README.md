# Multi camera tutorial

In this tutorial you will learn how to create a launch file to configure and start multiple ZED nodes 
for a multi-camera configuration.

## The launch file explained

The example launch file `zed_multi_camera.launch.py` allows to dynamically configure a robotics system equipped with more ZED camera of different models.

The number of ZED nodes to start is infered by the size of the parameters passed to the launch file from the command line. 

**Note:** The TF broadcasting is disabled for all the cameras, but the first which is normally configured to publish the `map` -> `odom` -> `base_link` TF chain for the full system, unless `disable_tf` is set to `False`.

All the parameter arrays must have the same size:

* `cam_names`: {REQUIRED} An array containing the names of the cameras, e.g. `'[zed_front,zed_back,zed_left, zed_right]'`. Note: it is important that all the cameras have a different name to correctly distinguish them.
* `cam_models`: {REQUIRED} An array containing the names of the cameras, e.g. `'[zed2i,zed2,zed2,zed2i]'`
* `cam_serials`: {REQUIRED} An array containing the serial numbers of the cameras, e.g. `[3001234,2001234,2004321,3004321]`
* `cam_poses`: {REQUIRED} An array containing the array of the pose of the cameras with respect to the base frame link, e.g. `[[0.5,0.0,0.0,0.0,0.0,0.0],[0.0,0.2,0.0,0.0,1.571,0.0]],[0.0,-0.2,0.0,0.0,-1.571,0.0],[-0.5,0.0,0.0,0.0,0.0,3.142]]]`
* `disable_tf`: Disable TF broadcasting for all the cameras in order to fuse visual odometry information externally. [default: 'false']

## Example

### Two cameras

Launch command for a robot configuration with two cameras, the second is placed on top of the first looking backward:

    $ ros2 launch zed_multi_camera zed_multi_camera.launch.py cam_names:='[zed_front,zed_back]' cam_models:='[zed2i,zed2]' cam_serials:='[35199186,23154724]' cam_poses:=[[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.03,0.0,0.0,3.142]]

### Four cameras

Launch command for a robot configuration with four cameras:

* The first camera is a ZED2i placed in the middle-front of the robot, and looking forward, i.e. `'zed_front','zed2i','30001234','[0.5,0.0,0.0,0.0,0.0,0.0]'`.
* The second camera is a ZED2 place in the middle-left side of the robot, and looking left, i.e. `'zed_left','zed2','20001234','[0.0,0.3,0.0,0.0,1.571,0.0]'`.
* The third camera is a ZED2 placed in the middle-right of the robot, and looking right, i.e. `'zed_right','zed2','20004321','[0.0,-0.3,0.0,0.0,-1.571,0.0]'`.
* The fourth camera is a ZED2i place in the middle-back of the robot, and looking back, i.e. `'zed_back','zed2i','30004321','[-0.5,0.0,0.0,0.0,0.0,3.142]'`.

    $ ros2 launch zed_multi_camera zed_multi_camera.launch.py cam_names:='[zed_front,zed_left, zed_right,zed_back]' cam_models:='[zed2i,zed2,zed2,zed2i]' cam_serials:='[30001234,20001234,20004321,30004321]' cam_poses:=[[0.5,0.0,0.0,0.0,0.0,0.0],[0.0,0.3,0.0,0.0,1.571,0.0],[0.0,-0.3,0.0,0.0,-1.571,0.0],[-0.5,0.0,0.0,0.0,0.0,3.142]]




