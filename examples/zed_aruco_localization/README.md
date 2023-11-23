# ZED ArUco Localization

This example demonstrates how to use a set of ArUco markers placed at known positions and orientations in the environment to locate a ZED camera device.

The example leverages the concept of [Node Composition in ROS 2(https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html)]. It creates a ROS 2 component named `stereolabs::ZedArucoLoc` that must be executed in the same container (process)where the `stereolabs::ZedCamera` component is running to leverage the Intra Process Communication behavior of ROS 2 and minimize the data latency.

The launch file `zed_aruco_loc.launch.py` shows how to start a ROS 2 container and load the `stereolabs::ZedCamera` component and the `stereolabs::ZedArucoLoc` component.

The `stereolabs::ZedArucoLoc` component subscribes to the rectified left camera RGB image topic of the component `stereolabs::ZedCamera`, performs ArUco detection and recalculates the pose of the camera according to the pose of the detected marker in the world. Finally, the `stereolabs::ZedArucoLoc` component calls the `set_pose` service provided by the `stereolabs::ZedCamera` component to fix the pose of the camera.

You can generate your ArUco markers by using [this online tool](https://chev.me/arucogen/). It is important to select the **6x6** dictionary.

## Set the parameters

The parameters are described in the file `config/aruco_loc.yaml` that is loaded by the launch file:

```
      general:
        marker_count: <N>           # Number of markers in the World
        marker_size: <S>            # Width/Height of the ArUco tags [m]
        maximum_distance: <D>       # Maximum distance of the target from the camera to consider it valid
        detection_rate: <F>         # Maximum detection frequency for camera pose update
        camera_name: "zed"          # Name of the camera to relocate
        world_frame_id: "map"       # Frame id of the world frame
        refine_detection: false     # If enabled the corners of the detected markers will be processed to obtain sub-pixel precision

      debug:
        active: false               # Enable debug messages
```

For each marker in the world (this must match the number in `marker_count`):

```
      marker_<idx>:
        aruco_id: <id>                      # ID of the ArUco tag as retrieved by the ArUco Detector code
        position: [<pos_X>,<pos_Y>,<pos_Z>] # Pose with respect to the World origin [m]
        orientation: [<roll>,<pitch>,<yaw>] # Orientation with respect to the World origin [rad]
```

> **Note:** the folder `markers` contains the SVG files of the markers **#19** and **#43** listed in the example file `aruco_loc.yaml`. 
You can use a vectorial editor (e.g. Inkscape) to resize them according to the value of the parameter `general.marker_size`.

## Run the example

Call the command 

`ros2 launch zed_aruco_localization zed_aruco_loc.launch.py camera_model:=<camera_model>` 

to start the `zed_aruco_localization` node.

> **Note:** you can add the option `rviz:=false` to disable the automatic load of RVIZ2.

The `zed_aruco_localization` node performs ArUco detection at a fixed rate by subscribing to ZED color image topics. When a tag is detected it calculates the camera pose with respect to it and calls the `set_pose` service of the ZED node to reset the camera pose in the "world".

> **Note:** By default, the gravity alignment is turned off. However, you can turn it on by using the launch option `gravity_alignment:=true`. Please note that for accurate ArUco re-location, it is crucial to ensure that the pitch and yaw values of each marker's orientation are properly set with respect to the Earth's gravity vector.

Before running the example, it is important to set the following parameters in `config/aruco_loc.yaml`:

* `general.marker_count`: indicates how many markers are placed in the world
* `general.marker_size`: dimension in meters of the size of the marker

Then for each marker a set of parameters must be added:

```yaml
      marker_xxx:
        aruco_id: yy                 # ID of the ArUco tag as retrieved by the ArUco Detector code
        position: [X,Y,Z]            # Pose with respect to the World origin [m]
        orientation: [R,P,Y]         # Orientation with respect to the World origin [rad]
```

* replace `marker_xxx` with an incremental index, i.e. `marker_000` for the first marker, `marker_001` for the second marker, and so on.
* `aruco_id`: the index of the marker as detected
* `position`: the position of the marker in world frame, i.e. with respect to the origin
* `orientation`: the roll, pitch, yaw orientation [rad] of the marker in world frame, i.e. with respect to the origin

> **Note**: use the command `ros2 launch zed_aruco_localization zed_aruco_loc.launch.py -s` to get the list of all the availabe launch  parameters.






