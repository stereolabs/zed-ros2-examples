# ZED Benchmark

This package contains a set of benchmarks to test the performances of the ZED ROS2 Wrapper

## Usage

``` bash
$ ros2 run zed_topic_benchmark zed_topic_benchmark --ros-args -p topic_name:=<name_of_the_topic_to_monitor>
```

The benchmark stops and prints a report when a duration or sample-count limit is reached, or when interrupted with `Ctrl+C`. For example, benchmark 1000 messages and save the report to a file:

``` bash
$ ros2 run zed_topic_benchmark zed_topic_benchmark --ros-args \
    -p topic_name:=<name_of_the_topic_to_monitor> \
    -p test_sample_count:=1000 \
    -p log_file_path:=/tmp/benchmark_report.txt
```

## Multi-camera launch

The `zed_test_ipc.launch.py` launch file starts a multi-camera setup and one benchmark node per camera (composed in the camera container, or as separate processes):

``` bash
$ ros2 launch zed_topic_benchmark zed_test_ipc.launch.py \
    cam_names:=[zed_front,zed_back] cam_models:=[zed2i,zed2] cam_serials:=[12345678,87654321]
```

Launch arguments:

* `cam_names`, `cam_models`, `cam_serials`: arrays describing the cameras to start.
* `disable_tf`: disable TF broadcasting for all cameras. [Default: `False`]
* `use_ipc`: load the benchmark nodes as components in the camera container, with intra-process comms enabled, instead of starting them as separate processes. [Default: `True`] The benchmark nodes are given `subscription_mode:=typed`, which is what makes the intra-process path usable — see [Measuring Intra Process Communication](../README.md#measuring-intra-process-communication).
* `subscription_mode`: subscription path used by the benchmark nodes: `auto`, `generic` or `typed`. Applied to both branches so that `use_ipc:=True` and `use_ipc:=False` are directly comparable. [Default: `typed`]
* `qos_reliability`: subscriber QoS reliability, `best_effort` or `reliable`. [Default: `best_effort`] Both work on ZED topics, which are published `Reliable`. A `Reliable` subscriber only fails against a `Best Effort` publisher, so keep the default when benchmarking a topic from some other node whose reliability you have not checked.
* `qos_durability`: subscriber QoS durability, `volatile` or `transient_local`. [Default: `volatile`]
* `qos_history`: subscriber QoS history, `keep_last` or `keep_all`. [Default: `keep_last`]
* `qos_depth`: subscriber QoS depth, used by `keep_last`. [Default: `1`]
* `avg_win_size`: window size of the running averages. [Default: `500`] Applied to both branches: the report min/max are tracked on the windowed average, so an asymmetric window would make the two `use_ipc` runs incomparable.
* `topic_name`: topic to benchmark, without the `/zed_multi/<cam_name>` prefix. [Default: `/point_cloud/cloud_registered`]
* `test_duration_sec`: benchmark duration in seconds. `0.0` runs until `Ctrl+C`. [Default: `0.0`]
* `test_sample_count`: number of messages to acquire before stopping. `0` runs until `Ctrl+C`. [Default: `0`]
* `log_file_path`: file where the report is saved (the camera name is appended to keep per-camera reports separate). Empty disables file logging. [Default: `""`]

**Note:** with `use_ipc:=True` all benchmark nodes share the camera container, so a finite `test_duration_sec`/`test_sample_count` will stop the whole container once the limit is reached. Keep the limits at their defaults and stop with `Ctrl+C` to benchmark composed nodes for an arbitrary time.
