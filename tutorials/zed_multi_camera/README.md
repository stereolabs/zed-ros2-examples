# Multi camera tutorial

In this tutorial you will learn how to create a launch file to configure and start multiple ZED nodes 
for a multi-camera configuration.

Example launch command for a rig made with two cameras:

    $ ros2 launch zed_multi_camera zed_multi_camera.launch.py cam_names:='["zed_front","zed_back"]' cam_models:='["zed2i","zed2"]' cam_serials:='["35199186","23154724"]' cam_poses:=[[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,3.414,0.0]]




