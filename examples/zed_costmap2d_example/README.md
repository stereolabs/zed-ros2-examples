# Nav2 2D Cost Map example

This example demonstrates how to configure the Nav2 package to generate a 2D cost map for robot navigation by using the point cloud published by the ZED ROS2 node and the Spatio Temporal Voxel Layer (STVL) plugin.

## Run the example

A launch file is provided to start the example in the default configuration:

`$ ros2 launch zed_costmap2d_example start_example.launch.py`

The launch file will start the ZED Node configured for a ZED2i camera, the Nav2 infrstructure in SLAM mode, and finally Rviz2 pre-configured for data visualization.

TODO add image

### Configuration

Run the command 

`$ ros2 launch zed_costmap2d_example start_example.launch.py -s`

for the list of all the available launch file options:

* `svo_path`: full path to a SVO file to be used as input instead of a live camera.
* `cam_pos_x`, `cam_pos_y`, `cam_pos_z`: X, Y, and Z coordinates of the camera with respect to the robot `base_link`. **Note**: by default the parameter `pos_tracking.floor_alignment` in `params/custom_common.yaml` is set to `true`, so the `cam_pos_z` value is ignored being automatically estimated by the node - Default: `0.0`.
* `cam_roll`, `cam_pitch`, `cam_yaw`: Roll, Pitch, and Yaw angles of the camera with respect to the robot `base_link` - Default: `0.0`.
* `params_file`: full path to the parameters file to use for all launched Nav2 nodes - Default: `params/nav2_custom_params.yaml`.
* `default_bt_xml_filename`: full path to the behavior tree xml file to use - Default: `nav2_bt_navigator/behavior_trees/navigate_w_replanning_distance.xml`.


