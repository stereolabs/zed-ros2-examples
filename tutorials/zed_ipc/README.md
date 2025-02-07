# ROS 2 Composition with Intra-Process Communication Tutorial

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
ros2 launch zed_ipc zed_ipc.launch.py
```

This command will start the launch file, initializing the ZED Camera nodes and the custom subscriber node within the same process.

### The code explained

[TODO]