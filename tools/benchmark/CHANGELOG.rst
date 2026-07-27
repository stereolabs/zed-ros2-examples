LATEST CHANGES
==============

v5.4.1
----------
- Added support for ROS 2 Lyrical Luth. The benchmark packages previously built only on Humble, Iron and Jazzy: on any other distribution they were silently configured as empty, so the workspace build succeeded but the message, the component library and the ``zed_topic_benchmark`` executable were never produced. The distribution check is now an exclusion list (only Foxy and older lack the "generic subscription" API this tool requires), so Lyrical, Rolling and future releases build without further changes. On an unsupported distribution the packages now emit an explicit warning and are still registered in the ament index.
- Added the ``zed_check_ros2_config.sh`` script to validate a ROS 2 setup. It benchmarks the ZED RGB image, depth and point cloud topics, in both standard and IPC (composition) mode, and prints a report for each test so the user can verify that their ROS 2 / DDS / system configuration delivers the camera data at the expected rate and bandwidth. Run it with ``ros2 run zed_topic_benchmark zed_check_ros2_config.sh <camera_model>``.
- Added the possibility to run the benchmark for a limited time or number of samples and to save a report:

  - New parameter ``test_duration_sec``: the test stops after the given number of seconds (``0`` = infinite, default).
  - New parameter ``test_sample_count``: the test stops after the given number of received messages (``0`` = infinite, default).
  - New parameter ``log_file_path``: when set, the final report is also written to this file.
  - When the test completes (a limit is reached) or is interrupted with ``Ctrl+C``, a summary report is printed to the console listing the topic name/type, the stop reason, the test duration, the number of received messages, the total received data, and the mean/min/max of frequency, message size and bandwidth over the whole test.

- Fixed the report *min/max* frequency and bandwidth, which showed meaningless extremes (e.g. a max frequency of hundreds of kHz) caused by a single short inter-arrival interval. They are now tracked on the windowed average rate, after the averaging window has filled.
- Improved the report and live message-size display to use adaptive units (B / KB / MB / GB) so small messages (e.g. IMU samples) are no longer rounded to ``0.00 MB``.
- Reworked the live console line into a compact, fixed-width ``instant/average`` format that fits within 80 columns. Previously the long line wrapped on a default-size terminal, which broke the in-place (carriage-return) update and flooded the console with one line per message. An ANSI "erase to end of line" is also emitted to clear any leftover characters.
- Fixed the windowed *average frequency* and *average bandwidth* statistics, which were biased high. The averages are now computed on the message periods and sizes (``1/mean(Δt)`` and ``mean(size)/mean(Δt)``), matching the unbiased windowed rate used by ``ros2 topic hz``. Reported averages were previously optimistic, increasingly so on jittery topics.
- Fixed wrong statistics when multiple benchmark components run in the same process via IPC composition (e.g. the multi-camera launch): a shared initialization flag caused every node but the first to emit one garbage sample. Each component now keeps its own state.
- Fixed *average bandwidth* on variable-size topics (compressed images, point clouds): it now uses the windowed mean message size instead of the latest message size.
- Changed the default subscriber QoS reliability to ``Best Effort`` so the tool connects to sensor-data topics (images, point clouds) out of the box. Override to ``Reliable`` at runtime with ``-p qos_overrides./<topic>.subscription.reliability:=reliable`` if needed.
- Fixed possible ``inf``/``NaN`` statistics caused by unmeasurable inter-arrival intervals and empty averaging windows.
- Switched interval timing to a monotonic clock (``steady_clock``) for correct measurements under host clock adjustments (NTP/PTP).
- Fixed the ``zed_test_ipc.launch.py`` launch file failing to start due to an accidental ``sympy`` import.
