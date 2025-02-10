# TUTORIAL - ROS 2 Composition with Intra-Process Communication

This tutorial demonstrates how to leverage [ROS 2 Composition](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html) and [Intra-Process Communication](https://docs.ros.org/en/humble/Tutorials/Demos/Intra-Process-Communication.html) to create a new node component that subscribes to point cloud topics published by all ZED Camera node components running within the same process.

The tutorial guides you through the creation of a new launch file. This launch file utilizes the launch file of the [`zed_multi_camera` example](tutorials/zed_multi_camera) to:

1. **Start multiple ZED nodes within a single process.**
2. **Load a new node component that subscribes to the point cloud topics.**
3. **Achieve zero-copy data transfer for efficient data exchange.**

This approach enhances performance and reduces resource consumption by minimizing data copying between nodes within the same process.

## `zed_ipc.launch.py` Launch File

The `zed_ipc.launch.py` launch file is designed to facilitate the setup of multiple ZED Camera nodes within a single process, along with a custom node component that subscribes to their point cloud topics. This launch file ensures efficient intra-process communication by leveraging zero-copy data transfer.

### Key Features

- **Multiple ZED Nodes**: Initializes and manages multiple ZED Camera nodes within the same process to capture point cloud data.
- **Custom Subscriber Node**: Loads a custom node component that subscribes to the point cloud topics published by the ZED Camera nodes.
- **Zero-Copy Data Transfer**: Utilizes ROS 2's intra-process communication capabilities to achieve zero-copy data transfer, enhancing performance and reducing resource usage.

### Usage

To use the `zed_ipc.launch.py` launch file, execute the following command:

```bash
ros2 launch zed_ipc zed_ipc.launch.py cam_names:=[<camera_name_array>] cam_models:=[<camera_model_array>] cam_serials:=[<camera_serial_array>]
```

This command will start the launch file, initializing the ZED Camera nodes and the custom subscriber node within the same process.

### The code explained

**First step**: create a ROS 2 Container and load the two ZED camera components nodes inside it. Here we use the multi-camera launch file described in the [multi-camera tutorial](../zed_multi_camera).

```python
    # Call the multi-camera launch file
    multi_camera_launch_file = os.path.join(
        get_package_share_directory('zed_multi_camera'),
        'launch',
        'zed_multi_camera.launch.py'
    )
    zed_multi_camera = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(multi_camera_launch_file),
        launch_arguments={
            'cam_names': names,
            'cam_models': models,
            'cam_serials': serials,
            'disable_tf': disable_tf
        }.items()
    )
    actions.append(zed_multi_camera)
```

**Second step**: remap the topic names. The demo node that receives the Point Cloud messages subscribes to generic `pointcloud_X` topic names.
The launch file must create a correct remapping between `pointcloud_X` and each ZED node Point Cloud topic name `/zed_multi/<camera_name/point_cloud/cloud_registered`.

```python
    # Create topic remappings for the point cloud node
    remappings = []
    name_array = parse_array_param(names.perform(context))
    for i in range(cam_count):
        base_topic = 'pointcloud_' + str(i)
        remap = '/zed_multi/' + name_array[i] + '/point_cloud/cloud_registered'
        remapping = (base_topic, remap)
        remappings.append(remapping)
```

**Third step**: create the demo component node that subscribes to the Point Cloud topics and processes the relative messages.

```python
    pc_node = ComposableNode(
        package='zed_ipc',
        plugin='stereolabs::PointCloudComponent',
        name='ipc_point_cloud',
        namespace='zed_multi',
        parameters=[{
            'cam_count': cam_count
        }],
        remappings=remappings,
        extra_arguments=[{'use_intra_process_comms': True}]
    )
```

**Final step**: load the Point Cloud component node in the existing ZED Container process to leverage Intra Process Communication.

```python
    load_pc_node = LoadComposableNodes(
        composable_node_descriptions=[pc_node],
        target_container='/zed_multi/zed_multi_container'
    )
    actions.append(load_pc_node)
```

You can verify that all the nodes are now running into the same container process by using the command `ros2 component list`.

#### Example

Here's an example for two ZED X cameras named `zedx_front` and `zedx_rear`:

- Launch command:

```bash
ros2 launch zed_ipc zed_ipc.launch.py cam_names:=[zedx_front,zedx_rear] cam_models:=[zedx,zedx] cam_serials:=[xxxxxxxxx,yyyyyyyy]
```

- Running check:

```bash
$ ros2 component list
/zed_multi/zed_multi_container
  1  /zed_multi/zedx_front
  2  /zed_multi/zedx_rear
  3  /zed_multi/ipc_point_cloud
```
