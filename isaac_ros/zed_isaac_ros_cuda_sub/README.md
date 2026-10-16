# ZED CUDA Zero-Copy Subscriber Example

This example subscribes to a ZED camera image topic using the CUDA zero-copy transport of ROS 2 Lyrical (`rosidl::Buffer` with the `cuda` buffer backend) and displays communication benchmarks.

The example runs two benchmark phases on the same topic and compares them:

1. **CPU**: the subscription accepts only CPU data (the ROS 2 default), so the image is copied from GPU to CPU memory before being delivered.
2. **CUDA**: the subscription accepts the `cuda` buffer backend, so the image stays in GPU memory and the subscriber reads it directly from the memory of the ZED node, without copies.

For each phase it reports the subscription frequency, the latency, and the CPU and GPU load.

## Requirements

- ROS 2 Lyrical with the `rmw_fastrtps_cpp` middleware (NVIDIA Isaac ROS 5 environment)
- ZED ROS 2 Wrapper v5.6.0 or newer, built with the `cuda_buffer` package
- The `cuda_buffer_backend` plugin

Details about the example, the installation, and the usage are available in the [Isaac ROS with ZED documentation](https://docs.stereolabs.com/isaac-ros).

## Usage

```bash
ros2 launch zed_isaac_ros_cuda_sub zed_cuda_sub_example.launch.py camera_model:=<camera_model>
```

Use the `topic_name` launch argument to select another ZED image topic, for example the depth map:

```bash
ros2 launch zed_isaac_ros_cuda_sub zed_cuda_sub_example.launch.py camera_model:=<camera_model> topic_name:=/zed_isaac/zed/depth/depth_registered
```
