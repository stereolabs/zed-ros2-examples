# ZED CUDA Zero-Copy Point Cloud Subscriber Example

This example subscribes to the ZED point cloud using the CUDA zero-copy transport of ROS 2 Lyrical (`rosidl::Buffer` with the `cuda` buffer backend), prints the coordinates of the point in the middle of the cloud, and displays communication benchmarks.

The example runs two benchmark phases on the same topic and compares them:

1. **CPU**: the subscription accepts only CPU data (the ROS 2 default), so the point cloud is copied from GPU to CPU memory before being delivered, and uploaded back to GPU memory by the subscriber.
2. **CUDA**: the subscription accepts the `cuda` buffer backend, so the point cloud stays in GPU memory and the subscriber reads it directly from the memory of the ZED node, without copies.

For each received point cloud it prints the X, Y, and Z coordinates, in meters, of the middle point, read from GPU memory. For each phase it reports the subscription frequency, the latency, and the CPU and GPU load.

## Requirements

- ROS 2 Lyrical with the `rmw_fastrtps_cpp` middleware (NVIDIA Isaac ROS 5 environment)
- ZED ROS 2 Wrapper v5.6.0 or newer, built with the `cuda_buffer` package
- The `cuda_buffer_backend` plugin

Details about the setup of the CUDA zero-copy transport are available in the [Isaac ROS with ZED documentation](https://docs.stereolabs.com/isaac-ros).

## Usage

```bash
ros2 launch zed_isaac_ros_cuda_pc_sub zed_cuda_pc_sub_example.launch.py camera_model:=<camera_model>
```

The example subscribes to `/zed_isaac/zed/point_cloud/cloud_registered`: use the `topic_name` launch argument to select another point cloud topic.

The coordinates are expressed in the frame of the point cloud message (`header.frame_id`). They are not valid (NaN) when the depth is not available for the middle point, for example when the object in front of the camera is too close or too far.
