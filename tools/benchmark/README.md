# Stereolabs ZED Camera - ROS 2 topics benchmark tool

This package provides a benchmark node to measure the frequency and bandwidth of ROS 2 topics and, optionally, to plot the values in real time.

For each topic the following information will be available:

* Topic type
* Real-time frequency
* Windowed average frequency
* Topic size
* Real-time topic bandwidth
* Windowed average topic bandwidth

The node publishes a message on the topic `<name_of_the_topic_to_test>_stats` containing all the information, so that it can optionally be plotted.

**Note:** This tool is available for Humble, Jazzy, Lyrical and newer distributions. It is not available for Foxy and older distributions because they lack an important feature required to subscribe to "generic topics".

## Usage

Open a new terminal console and start the benchmark node:

```bash
ros2 run zed_topic_benchmark zed_topic_benchmark --ros-args -p topic_name:=<name_of_the_topic_to_test>
```

For example:

```bash
ros2 run zed_topic_benchmark zed_topic_benchmark --ros-args -p topic_name:=/zed2i/zed_node/rgb/color/rect/image
```

By default the benchmark runs until you stop it with `Ctrl+C`. You can instead limit it by time or by number of messages, and save the final report to a file.

Run for 30 seconds, then stop and print the report:

```bash
ros2 run zed_topic_benchmark zed_topic_benchmark --ros-args \
  -p topic_name:=/zed2i/zed_node/rgb/color/rect/image \
  -p test_duration_sec:=30.0
```

Run until 1000 messages are received, then stop:

```bash
ros2 run zed_topic_benchmark zed_topic_benchmark --ros-args \
  -p topic_name:=/zed2i/zed_node/rgb/color/rect/image \
  -p test_sample_count:=1000
```

Run for 60 seconds and also save the report to a log file:

```bash
ros2 run zed_topic_benchmark zed_topic_benchmark --ros-args \
  -p topic_name:=/zed2i/zed_node/rgb/color/rect/image \
  -p test_duration_sec:=60.0 \
  -p log_file_path:=/tmp/zed_benchmark_report.txt
```

The node will print all the topic information on the console:

```bash
[INFO] [1665764376.862143962] [topic_benchmark]: ***** Benchmark parameters *****
[INFO] [1665764376.862355022] [topic_benchmark]: * Topic name: /zed/zed_node/rgb/color/rect/image
[INFO] [1665764376.862462857] [topic_benchmark]: Average window size: 500
[INFO] [1665764376.862496172] [topic_benchmark]: *** START BENCHMARK ***
[INFO] [1665764376.863484009] [topic_benchmark]: Advertised on topic: /zed/zed_node/rgb/color/rect/image_stats
[INFO] [1665764377.363430211] [topic_benchmark]: Found topic: '/zed/zed_node/rgb/color/rect/image' of type: 'sensor_msgs/msg/Image'
#119   | Freq   6.01/ 13.49 Hz | BW   42.28/ 94.87 Mbps | 901.12 KB
```

The live line is updated in place and shows, for frequency and bandwidth, the `instant/average` values. It is kept compact (under 80 columns) so it does not wrap on a default terminal.

## Parameters

* `topic_name`: name of the topic to test
* `avg_win_size`: size of the mobile window for the calculation of the average. [Default: `500`]
* `test_duration_sec`: duration of the test in seconds. The test stops and a report is generated once this time has elapsed (counted from the first received message). `0` means run until interrupted. [Default: `0` → infinite]
* `test_sample_count`: number of messages to acquire before stopping the test and generating the report. `0` means run until interrupted. [Default: `0` → infinite]
* `log_file_path`: path of a file where the final report is written (in addition to the console). Empty means console only. [Default: `""`]
* `use_ros_log`: if `true`, prints the live statistics and report through the ROS logging system instead of the console. [Default: `false`]

When both `test_duration_sec` and `test_sample_count` are set, the test stops as soon as the first of the two limits is reached.

## Final report

When the test completes (a duration or sample-count limit is reached) **or** when it is interrupted by the user with `Ctrl+C`, a summary report is printed to the console — and written to `log_file_path` if set — containing the topic name and type, the stop reason, the test duration, the number of received messages, the total received data, and the mean / min / max of frequency, message size and bandwidth over the whole test:

```text
================ ZED TOPIC BENCHMARK REPORT ================
Topic name:        /zed/zed_node/point_cloud/cloud_registered
Topic type:        sensor_msgs/msg/PointCloud2
Stop reason:       test completed (sample count reached)
Test duration:     10.00 s
Messages received: 150
-----------------------------------------------------------
Frequency [Hz]   - mean: 15.00 | min: 14.82 | max: 15.13
Msg size         - mean: 3.93 MB | min: 3.93 MB | max: 3.93 MB
Bandwidth [Mbps] - mean: 471.62 | min: 465.74 | max: 475.49
Total data:        589.82 MB
===========================================================
```

## Custom message

The node publishes a message on the topic `<name_of_the_topic_to_test>_stats` containing the information described above, so that it can be plotted.
The message is a custom type, `BenchmarkStatsStamped`, defined as:

``` ros
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

## QoS

The benchmark subscriber uses a `Best Effort`, `KEEP_LAST` (depth 1) QoS by default. A `Best Effort` subscriber is compatible with both `Reliable` and `Best Effort` publishers, so it works out of the box with sensor-data topics (images, point clouds). If you specifically need `Reliable` reliability you can override it at runtime:

```bash
ros2 run zed_topic_benchmark zed_topic_benchmark --ros-args \
  -p topic_name:=<topic> \
  -p qos_overrides./<topic>.subscription.reliability:=reliable
```

## Note on the statistics

The *instant* frequency/bandwidth are computed from the last inter-arrival interval. The *average* values are the windowed mean **period** and **message size** inverted/combined (i.e. `1/mean(Δt)` and `mean(size)/mean(Δt)`), which is the unbiased windowed rate used by `ros2 topic hz`. Averaging the instantaneous rates directly would overestimate the true average frequency.

In the final report, the **mean** frequency/bandwidth are computed over the whole test (total messages / total time and total data / total time). The **min/max** are tracked on the *windowed average* rate — not on the raw single-sample instantaneous rate — and only after the averaging window has filled. This avoids reporting meaningless extremes caused by a single short inter-arrival interval (e.g. two messages delivered back-to-back by the executor or a publisher burst), which would otherwise show up as an enormous instantaneous frequency. Message sizes are printed with adaptive units (B / KB / MB / GB) so small messages are not rounded to `0.00 MB`.

## Advanced - using IPC and composition

The package provides a ROS 2 component called `stereolabs::TopicBenchmarkComponent` to be used with [Composition](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html) to test [Intra Process Communication (IPC)](https://design.ros2.org/articles/intraprocess_communications.html) performance.

**Note:** when several benchmark components run in the same process via IPC composition, setting a finite `test_duration_sec`/`test_sample_count` will shut the whole container down once a limit is reached (the report of each component is still produced). Leave the limits at their default (infinite) and stop the container with `Ctrl+C` to benchmark composed nodes for an arbitrary time.

## How to use this tool to test your ROS 2 configuration

The package ships a helper script, `zed_check_ros2_config.sh`, that runs a fixed set of benchmarks against a **real ZED node** and prints a report for each one. Its purpose is to **verify that your ROS 2 / DDS / system configuration is able to deliver the camera data at the expected rate and bandwidth** — a quick way to validate a new setup or to investigate performance problems.

The script starts the ZED node and benchmarks the following topics, each in both **standard** (the benchmark runs as a separate process) and **IPC** (the benchmark is loaded as a component in the same container as the ZED node, with Intra Process Communication) mode:

| Topic | Depth mode |
|-------|------------|
| `/zed/zed_node/rgb/color/rect/image` | `NONE` |
| `/zed/zed_node/depth/depth_registered` | `NEURAL_LIGHT` |
| `/zed/zed_node/point_cloud/cloud_registered` | `NEURAL_LIGHT` |

The RGB image is tested with depth disabled (`NONE`), while the depth and point cloud topics are tested with the default depth mode (`NEURAL_LIGHT`).

### Usage

```bash
ros2 run zed_topic_benchmark zed_check_ros2_config.sh <camera_model> [duration_sec]
```

For example, for a ZED 2i with 20 seconds of measurement per test:

```bash
ros2 run zed_topic_benchmark zed_check_ros2_config.sh zed2i 20
```

The camera model is required (e.g. `zed`, `zed2`, `zed2i`, `zedx`, `zedxm`, ...). The behavior can be further tuned through environment variables: `CAMERA_MODEL`, `DURATION`, `WIN_SIZE`, `TOPIC_TIMEOUT`, `SETTLE` and `OUTPUT_DIR` (see the header of the script for details).

A full report (see [Final report](#final-report)) is saved for each test in the output directory, and a summary table is printed at the end:

```text
############################# SUMMARY #############################
TOPIC    MODE      MSGS      FREQ[Hz]     BW[Mbps]
-------------------------------------------------------------------
image    standard  595       29.78        1953.21
depth    standard  448       14.92        977.10
cloud    standard  447       14.90        477.30
image    ipc       598       29.92        1962.45
depth    ipc       450       15.01        982.55
cloud    ipc       451       15.04        481.92
-------------------------------------------------------------------
```

### When the results are not as expected

If the measured frequencies/bandwidths are lower than expected, or if IPC mode does not improve on the standard mode, your ROS 2 middleware or system is likely not tuned for high-throughput data. Refer to the online documentation:

* ROS 2 Documentation: <https://docs.stereolabs.com/docs/integrations/ros-2>
* DDS and Network Tuning for ROS 2: <https://docs.stereolabs.com/docs/integrations/ros-2/dds-and-network-tuning>
* Stereo Node Frequency Tuning: <https://docs.stereolabs.com/docs/integrations/ros-2/node-frequency-tuning>

## Package Testing

The statistics engine (sliding-window averages and the frequency/bandwidth math) is covered by C++ unit tests, and the end-to-end behavior (report generation and self-termination on a sample-count limit) is covered by a `launch_testing` integration test that runs the node against a synthetic publisher.

> **Important:** always run `colcon` from the root of your ROS 2 workspace (the folder containing `src/`, `build/`, `install/`). Running it from another directory makes `colcon` scan that directory recursively, which can pull in unrelated `setup.py` files (e.g. under a home folder) and abort with package-identification errors:
>
> ```bash
> cd ~/ros2_ws   # your workspace root
> colcon test --packages-select zed_topic_benchmark
> ```

Build with testing enabled and run them with:

```bash
# Unit tests (statistics engine)
colcon build --packages-select zed_topic_benchmark_component --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select zed_topic_benchmark_component --ctest-args -R "test_winavg|test_statistics"

# Integration test (node + synthetic publisher)
colcon build --packages-select zed_topic_benchmark --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select zed_topic_benchmark --ctest-args -R "test_benchmark_integration"

# Show detailed results
colcon test-result --all --verbose
```

The integration test can also be run directly with:

```bash
launch_test src/zed-ros2-examples/tools/benchmark/zed_benchmark/test/test_benchmark_integration.py
```
