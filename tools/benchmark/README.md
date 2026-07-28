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
* `subscription_mode`: which subscription path to use — `auto`, `generic` or `typed`. [Default: `auto`] See [Measuring Intra Process Communication](#measuring-intra-process-communication).

When both `test_duration_sec` and `test_sample_count` are set, the test stops as soon as the first of the two limits is reached.

## Final report

When the test completes (a duration or sample-count limit is reached) **or** when it is interrupted by the user with `Ctrl+C`, a summary report is printed to the console — and written to `log_file_path` if set. Besides the statistics, it always states which subscription path was used, which delivery path was measured, and how the reported sizes must be read, so a stored report cannot be misinterpreted later:

```text
================ ZED TOPIC BENCHMARK REPORT ================
Topic name:        /zed/zed_node/point_cloud/cloud_registered
Topic type:        sensor_msgs/msg/PointCloud2
Subscription:      generic (runtime-typed)
Delivery path:     inter-process (middleware) - a generic subscription can never take the intra-process path
Size semantics:    serialized wire bytes
Stop reason:       test completed (sample count reached)
Test duration:     10.00 s
Messages received: 150
-----------------------------------------------------------
Frequency [Hz]   - mean: 15.00 | min: 14.82 | max: 15.13
Msg size         - mean: 3.93 MB | min: 3.93 MB | max: 3.93 MB
Bandwidth [Mbps] - mean: 471.62 | min: 465.74 | max: 475.49
Total data:        589.82 MB
Latency [ms]     - not available on this subscription path
Process CPU:       1.240 s (12.40% of one core)
                   whole process, including any other component loaded in it
===========================================================
```

The same topic benchmarked with a zero-copy subscription composed in the camera container:

```text
Subscription:      type-adapted (zero-copy, sl::Mat by pointer)
Delivery path:     intra-process, zero-copy - CONFIRMED (publisher's buffer received by pointer)
Size semantics:    message content bytes (no CDR framing) - NOT comparable to wire bytes
...
Latency [ms]     - mean: 0.02 | min: 0.01 | max: 0.05 (60 samples)
Process CPU:       0.010 s (0.34% of one core)
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

# Instant end-to-end latency [msec], 0.0 when unavailable
float32 topic_latency
# Average end-to-end latency [msec], 0.0 when unavailable
float32 topic_avg_latency
```

The two latency fields are `0.0` on the `generic` path, which has no directly usable publisher timestamp.

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

## Measuring Intra Process Communication

The package provides a ROS 2 component, `stereolabs::TopicBenchmarkComponent`, that can be loaded into a component container with [Composition](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html) so the benchmark runs in the publisher's process. Composing it is what makes [Intra Process Communication (IPC)](https://design.ros2.org/articles/intraprocess_communications.html) measurable — but it is **not sufficient on its own**, and the reason drives the design of this tool.

### Why a generic subscription can never measure IPC

By default the benchmark subscribes to a topic whose type is discovered at runtime, which requires an `rclcpp::GenericSubscription`. In rclcpp, a subscription is registered with the `IntraProcessManager` **only** from the constructor of the *templated* `rclcpp::Subscription<T>` (through `resolve_use_intra_process()` / `setup_intra_process()`). `GenericSubscription` derives directly from `SubscriptionBase` and never does that — on every distribution from Humble to Rolling. There is also no serialized intra-process path to opt into: `IntraProcessManager` has no notion of `SerializedMessage` at all.

So a generic subscription always receives through the middleware, even inside the publisher's own container with `use_intra_process_comms:=true`. Worth spelling out, because it is genuinely confusing: the ZED node can correctly log `[IPC type-adapted zero-copy]` while this tool measures the inter-process path, since with IPC enabled the publisher feeds *both* paths and only a real intra-process subscriber benefits.

### The `subscription_mode` parameter

| Mode | Subscription | Can take the intra-process path | Message size reported |
| ------ | ------------ | ------------------------------ | --------------------- |
| `generic` | `rclcpp::GenericSubscription` (any type) | No, ever | Exact serialized **wire bytes** |
| `typed` | `rclcpp::Subscription<T>` (supported types only) | Yes | **Message content bytes** |
| `auto` (default) | `typed` when intra-process comms are enabled on the node **and** the type is supported, `generic` otherwise | When it resolves to `typed` | Depends on the resolved path |

`auto` is chosen so that an ordinary separate-process run keeps the historical generic subscription, and therefore keeps reporting wire-accurate bandwidth exactly as before, while a composed run with IPC enabled automatically gets a subscription that can actually use it.

### Supported message types

`image_transport` and `point_cloud_transport` publish the same image or cloud once per transport plugin, each on its own sub-topic and with its own message type. All of them are supported:

| Transport | Topic | Message type | Reported size |
| --- | --- | --- | --- |
| `image_transport` raw | `<base>` | `sensor_msgs/msg/Image` | `data` |
| `image_transport` compressed | `<base>/compressed` | `sensor_msgs/msg/CompressedImage` | `data` |
| `image_transport` compressedDepth | `<base>/compressedDepth` | `sensor_msgs/msg/CompressedImage` | `data` |
| `image_transport` zstd | `<base>/zstd` | `sensor_msgs/msg/CompressedImage` | `data` |
| `image_transport` theora | `<base>/theora` | `theora_image_transport/msg/Packet` | `data` |
| `image_transport` ffmpeg | `<base>/ffmpeg` | `ffmpeg_image_transport_msgs/msg/FFMPEGPacket` | `data` |
| `point_cloud_transport` raw | `<base>` | `sensor_msgs/msg/PointCloud2` | `data` |
| `point_cloud_transport` draco / zlib / zstd / cloudini | `<base>/<transport>` | `point_cloud_interfaces/msg/CompressedPointCloud2` | `compressed_data` |
| — | — | `sensor_msgs/msg/CameraInfo` | matrices + distortion |
| — | — | `sensor_msgs/msg/Imu` | fixed fields |

For a compressed cloud the **compressed** payload is reported, not the uncompressed geometry: the latter would overstate the transported volume by the whole compression ratio.

The transport plugins are separate, individually installable packages, so `theora_image_transport`, `ffmpeg_image_transport_msgs` and `point_cloud_interfaces` are **optional build dependencies** — each type is compiled in only when its package is present, and the build logs which ones it enabled. A type that is not compiled in, or any type not in this table, falls back to `generic` with a warning saying so. Note that `sensor_msgs/msg/CompressedImage` covers three image transports and needs no extra package at all.

### Two tiers of typed delivery

Not all intra-process delivery is zero-copy, and the difference is large:

* **Plain typed subscription** — skips serialization and the middleware entirely, but rclcpp still copies the message into the subscription's buffer. When the publisher is type-adapted, rclcpp first calls `convert_to_ros_message()` and then copies that result into a `shared_ptr`, i.e. *two* full copies of the image.
* **ZED type-adapted subscription** — built on the very same `TypeAdapter<StampedSlMat, sensor_msgs::msg::Image>` the ZED node publishes with, so the publisher's `sl::Mat` arrives **by pointer**: no serialization, no conversion, no copy. This is genuine zero-copy, and it is selected automatically for `sensor_msgs/msg/Image` topics when intra-process comms are enabled.

The type-adapted tier needs `zed_components` and the ZED SDK at build time. The dependency is **optional**: without it the benchmark still builds and still measures the plain typed intra-process path, it just cannot report true zero-copy. The build prints which of the two it configured.

### What to compare — not bandwidth

On the intra-process path no bytes are transported, so a "bandwidth" figure there is notional payload throughput and is **not** comparable to an inter-process one. Frequency is set by the publisher and barely moves either. The metrics that actually show the gain are **latency** and **CPU**, both of which the report now includes.

Measured on a **ZED 2i**, `rgb/color/rect/image` (3.52 MB, ~1.7 Gbps) at 60 Hz with `NEURAL_LIGHT` depth running, on an RTX 4070 desktop. Both runs used `subscription_mode:=typed`, in the same camera session:

| | composed, zero-copy | separate process |
| --- | --- | --- |
| Latency mean | **18.06 ms** | **20.17 ms** |
| Total CPU (all processes) | **89.4%** of one core | **93.6%** (87.4 ZED + 6.2 benchmark) |
| ZED node with no subscriber | 70.0% | 70.0% |
| Frequency | 59.87 Hz | 59.76 Hz |

So on this machine zero-copy saves about **2 ms of latency and 4 points of CPU** on a 1.7 Gbps stream. The gain looks modest because both figures are dominated by work that is not the transport: 70 of those CPU points are capture plus depth, and ~18 ms of the latency is the camera pipeline (see [Reading the latency correctly](#reading-the-latency-correctly)). Isolating the transport with a synthetic type-adapted publisher — one that stamps at publish time and does no camera work — the same code reports **0.02 ms vs 3.32 ms** and 0.34% vs 2.04% CPU.

#### Reading the latency correctly

Latency runs from the publisher-side `header.stamp` to the arrival in the benchmark callback, so it needs a publisher that fills the stamp. It is reported as *not available* on the `generic` path, which has no directly usable timestamp.

> **The ZED node stamps images and clouds with the frame ACQUISITION time** (`sl::TIME_REFERENCE::IMAGE`), not the publish time. The reported latency therefore covers the *whole* pipeline — capture, USB transfer, SDK retrieve, rectification/depth, publish, deliver — and the transport is only a small part of it. Measured on a ZED 2i at 60 Hz, the rectified RGB image reports ~18–20 ms in **both** modes, of which only ~2 ms is the transport. So compare the *difference* between the two modes, not the absolute value.
>
> Two useful cross-checks from the same camera: `rgb/color/rect/camera_info` reports **0.09 ms**, because a `CameraInfo` is built at publish time and so times the transport alone; and `depth/depth_registered/compressedDepth` reports **53.9 ms**, because the stamp predates the compression the transport plugin then performs. Set the wrapper's `use_pub_timestamps` parameter to `true` to make every topic time the transport alone.

#### Reading the CPU figure correctly

The report's `Process CPU` covers the **whole process**. Composed, that process *is* the ZED node, so the figure also includes capture, depth and publishing — it is therefore **not comparable** to the separate-process figure, which covers the benchmark alone. On a ZED 2i this reads 6.2% separate vs 89.4% composed, which naively suggests IPC is far worse while in fact the totals are the other way around.

The fair comparison is the total across every process involved, which is what `zed_check_ros2_config.sh` reports in its `CPU_TOT` column (ZED node + benchmark for `interprocess`, the ZED node alone for `ipc`), alongside a `CPU_IDLE` no-subscriber baseline so the transport cost can be read as `CPU_TOT - CPU_IDLE`.

### What the report claims, and what it does not

A typed subscription on a node with IPC enabled takes the intra-process path *only* for publishers that live in the same process, and that cannot be checked from inside a callback. The report therefore distinguishes:

* `intra-process, zero-copy - CONFIRMED` — only the type-adapted path can prove this, because the custom C++ type it receives has no wire representation and so cannot have come through the middleware.
* `intra-process capable, NOT confirmed` — typed subscription, IPC enabled, but delivery is not verifiable per message. Compare the latency against an inter-process run to see which path you got.
* `inter-process (middleware)` — with the reason, including the case where a generic subscription makes the intra-process path impossible by construction.

**Note:** when several benchmark components run in the same process, setting a finite `test_duration_sec`/`test_sample_count` will shut the whole container down once a limit is reached (the report of each component is still produced). Leave the limits at their default (infinite) and stop the container with `Ctrl+C` to benchmark composed nodes for an arbitrary time.

## How to use this tool to test your ROS 2 configuration

The package ships a helper script, `zed_check_ros2_config.sh`, that runs a fixed set of benchmarks against a **real ZED node** and prints a report for each one. Its purpose is to **verify that your ROS 2 / DDS / system configuration is able to deliver the camera data at the expected rate and bandwidth** — a quick way to validate a new setup or to investigate performance problems.

The script starts the ZED node and benchmarks the following topics **twice** each — once with the benchmark in a separate process (`interprocess`) and once composed in the camera container with intra-process comms enabled (`ipc`) — always with a typed subscription, so the two runs use identical size and latency accounting and really are comparable. The ZED node keeps `enable_ipc:=true` in both, so the publisher is the same and the only variable is where the subscriber lives.

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

The layout of the summary (the values below are placeholders showing the format, not measurements — the numbers depend entirely on your camera, resolution, DDS and machine):

```text
############################# SUMMARY #############################
TOPIC   MODE          MSGS    FREQ[Hz]   LATENCY[ms]  CPU_TOT[%]  CPU_IDLE[%]
---------------------------------------------------------------------------------
image   interprocess  <n>     <freq>     <lat>        <total>     <idle>
image   ipc           <n>     <freq>     <lat>        <total>     <idle>
depth   interprocess  <n>     <freq>     <lat>        <total>     <idle>
depth   ipc           <n>     <freq>     <lat>        <total>     <idle>
cloud   interprocess  <n>     <freq>     <lat>        <total>     <idle>
cloud   ipc           <n>     <freq>     <lat>        <total>     <idle>
---------------------------------------------------------------------------------
```

`CPU_TOT` is the total across every process involved, so the two modes are directly comparable; `CPU_IDLE` is the ZED node with no subscriber, so the transport cost of a mode is `CPU_TOT - CPU_IDLE`. Expect the two `FREQ` values of a topic to be close (the publisher sets the rate) and `LATENCY`/`CPU_TOT` to differ. For actual measured values see [What to compare — not bandwidth](#what-to-compare--not-bandwidth).

Compare **LATENCY** and **CPU** between the two modes: those are what the intra-process path changes. Frequency is set by the publisher, and the bandwidth of an intra-process run is notional because nothing is transported — see [What to compare — not bandwidth](#what-to-compare--not-bandwidth).

### When the results are not as expected

If the measured frequencies/bandwidths are lower than expected, or the `interprocess` latency is high, your ROS 2 middleware or system is likely not tuned for high-throughput data. Refer to the online documentation:

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
