# Stereolabs ZED Camera - ROS 2 topics benchmark tool

This package provides benchmarking tools for evaluating the performance of ZED ROS 2 integrations. It includes two benchmark nodes: `zed_topic_benchmark`, which measures the throughput and stability of published ZED topics, and `zed_hardware_load_benchmark`, which monitors system resource usage such as CPU and GPU while the ZED pipeline is running. Together, these nodes help assess both communication performance and hardware load under realistic operating conditions.

## ZED Topic Benchmark node

It can be used to test frequency and bandwidth of topics and eventually plot the realtime values.

For each topic the following information will be available:

* Topic type
* Real time frequency
* Window average frequency
* Topic size
* Real time topic bandwidth
* Window average topic bandwidth


The node publishes a message on the topic `<name_of_the_topic_to_test>_stats` containing all the information to be eventually plotted.

**Note:** This tool is not available for Foxy distribution because it misses an important feature required to subscribe to "generic topics".

## Usage

Open a new terminal console and start the benchmark node:

```bash
$ ros2 run zed_benchmark zed_topic_benchmark --ros-args -p topic_name:=<name_of_the_topic_to_test>
```

for example:

```bash
ros2 run zed_benchmark zed_topic_benchmark --ros-args -p topic_name:=/zed2i/zed_node/rgb/color/rect/image
```

The node will print all the topic information on the console:

```bash
[INFO] [1665764376.862143962] [topic_benchmark]: ***** Benchmark parameters *****
[INFO] [1665764376.862355022] [topic_benchmark]: * Topic name: /zed/zed_node/rgb/color/rect/image
[INFO] [1665764376.862462857] [topic_benchmark]: Average window size: 500
[INFO] [1665764376.862496172] [topic_benchmark]: *** START BENCHMARK ***
[INFO] [1665764376.863484009] [topic_benchmark]: Advertised on topic: /zed/zed_node/rgb/color/rect/image_stats
[INFO] [1665764377.363430211] [topic_benchmark]: Found topic: '/zed/zed_node/rgb/color/rect/image' of type: 'sensor_msgs/msg/Image'
#119 - Freq: 6.01 Hz (Avg: 13.49 Hz) - Bandwidth: 42.28 Mbps (Avg: 94.87 Mbps) - Msg size: 0.88 MB
```

## Parameters

* `topic_name`: name of the topic to test
* `avg_win_size`: size of the mobile window for the calculation of the average. [Default: `500`]

## Custom message

The node published a message on the topic `<name_of_the_topic_to_test>_stats` containing the previous information to be plotted.
The topic is a custom message of type `BenchmarkStatsStamped` defined as:

```
# Standard Header
std_msgs/Header header

# Instant Frequency
float32 topic_freq
# Average Frequency
float32 topic_avg_freq

# Instant Bandwidth
float32 topic_bw
# Average Bandwidth
float32 topic_avg_bw
```
## ZED Topic Benchmark node


## Advanced
The package provides a ROS 2 component called `stereolabs::TopicBenchmarkComponent` to be used with [Composition](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html) to test [Intra Process Communication (IPC)](https://design.ros2.org/articles/intraprocess_communications.html) performances.