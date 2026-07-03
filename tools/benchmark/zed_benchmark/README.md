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

The `zed_test_ipc.launch.py` launch file starts a multi-camera setup and one benchmark node per camera (in IPC composition or as separate processes):

``` bash
$ ros2 launch zed_topic_benchmark zed_test_ipc.launch.py \
    cam_names:=[zed_front,zed_back] cam_models:=[zed2i,zed2] cam_serials:=[12345678,87654321]
```

Launch arguments:

* `cam_names`, `cam_models`, `cam_serials`: arrays describing the cameras to start.
* `disable_tf`: disable TF broadcasting for all cameras. [Default: `False`]
* `use_ipc`: load the benchmark nodes in the camera container using Intra Process Communication. [Default: `True`]
* `topic_name`: topic to benchmark, without the `/zed_multi/<cam_name>` prefix. [Default: `/point_cloud/cloud_registered`]
* `test_duration_sec`: benchmark duration in seconds. `0.0` runs until `Ctrl+C`. [Default: `0.0`]
* `test_sample_count`: number of messages to acquire before stopping. `0` runs until `Ctrl+C`. [Default: `0`]
* `log_file_path`: file where the report is saved (the camera name is appended to keep per-camera reports separate). Empty disables file logging. [Default: `""`]

**Note:** in IPC mode all benchmark nodes share the camera container, so a finite `test_duration_sec`/`test_sample_count` will stop the whole container once the limit is reached. Keep the limits at their defaults and stop with `Ctrl+C` to benchmark composed nodes for an arbitrary time.