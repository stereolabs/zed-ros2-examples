# Multi camera tutorial

In this tutorial you will learn how to create a multi-camera configuration with ZED devices.

For a multi-camera configuration it is important to correctly identify each camera in the system by
using its serial number and precisely set the position and the orientation of each of them with respect to a reference point.

## The launch file explained

The example launch file `zed_multi_camera.launch.py` allows to dynamically configure a robotics system equipped with multiple different models of ZED cameras 

The launch file starts a Robot State Publisher node that defines the position and orientation of each camera in the multi-camera system,  a Robot State Publisher node for each camera that broadcast all the static frames of the camera, and a ZED node for each camera.

The number of ZED nodes to start is infered by the size of the parameters passed to the launch file from the command line.

### Launch parameters

All the parameter arrays must have the same size:

* `cam_names`: {REQUIRED} An array containing the name of the cameras, e.g. `'[zed_front,zed_back]'`. **Note:** it is important that all the cameras have a different name to correctly distinguish them.
* `cam_models`: {REQUIRED} An array containing the model of the cameras, e.g. `'[zed2i,zed2]'`
* `cam_serials`: {REQUIRED} An array containing the serial numbers of the cameras, e.g. `[3001234,2001234]`
* `disable_tf`: Only the first camera is configured to broadcast the TF `map` -> `odom` -> `camera_link` to avoid TF conflicts. The TF broadcasting can be disabled by setting this parameter to `false`. This is required is an external Kalman filter that fuses different odometry sources is used.

## Run the example

Launch the dual camera nodes by using the following command

```bash
$ ros2 launch zed_multi_camera zed_multi_camera.launch.py cam_names:='[zed_front,zed_back]' cam_models:='[zed2i,zed2]' cam_serials:='[31234567,21234567]'
```

**Info:** to retrieve the serial number of each connected camera you can use the command `$ ZED_Explorer --all`.

**Note:** the parameters `cam_models` and `cam_serials` must be modified according to the configuration of your system.

Example:

```bash
$ ros2 launch zed_multi_camera zed_multi_camera.launch.py cam_names:='[zed_front,zed_back]' cam_models:='[<front_camera_model>,<rear_camera_model>]' cam_serials:='[<front_camera_serial>,<rear_camera_serial>]'
```

### The Multi Camera URDF

It is important to create an URDF that defines the position of all the cameras with respect to a reference link.

We provide an example in 'urdf/zed_multi.urdf.xacro'.

The URDF is created by using the 'xacro' tool.

First of all all reference links are created:

```xml
  <link name="$(arg multi_link)" />
  <link name="$(arg camera_name_0)_camera_link" />
  <link name="$(arg camera_name_1)_camera_link" />
```

The name of the reference link is contained in the xacro argument `multi_link`, the default value is `zed_multi_link`.
A "virtual" link `camera_link` for each camera must be defined to correctly connect the URDF of each camera to the reference link.

Next a joint to connect the reference link to the main link of each camera is created:

```xml
  <joint name="$(arg camera_name_0)_camera_joint" type="fixed">
      <parent link="$(arg camera_name_0)_camera_link"/>
      <child link="$(arg multi_link)"/>
      <origin xyz="0.06 0.0 0.0" rpy="0 0 0" />
  </joint>

  <joint name="$(arg camera_name_1)_camera_joint" type="fixed">
      <parent link="$(arg multi_link)"/>
      <child link="$(arg camera_name_1)_camera_link"/>
      <origin xyz="-0.06 0.0 0.0" rpy="0 0 ${M_PI}" />
  </joint>
```

The name of the joint is created from the variable `camera_name_x` that identifies the name of each camera.
A position and orientation must be provided for each camera with respect to the main link:
`<origin xyz="x y z" rpy="r p y" />` 
the position is in meters, the orientation in radians.

**Important:** the first joint has `parent link` and `child link` inverted with respect to all the other joints. This is required because
the first camera is the reference for visual odometry processing and in ROS a joint cannot have tro parents.

The following image display the TF tree generated for the dual camera configuration of this tutorial:

![](./images/frames.png)









