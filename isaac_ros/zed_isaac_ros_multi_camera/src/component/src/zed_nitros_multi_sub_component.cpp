// Copyright 2025 Stereolabs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "zed_nitros_multi_sub_component.hpp"

namespace stereolabs
{

ZedNitrosMultiSubComponent::ZedNitrosMultiSubComponent(const rclcpp::NodeOptions & options)
: Node("zed_nitros_sub_component", options), _defaultQoS(10)
{
  RCLCPP_DEBUG(get_logger(), "=======================================");
  RCLCPP_DEBUG(get_logger(), " ZED Nitros Multi Subscriber Component");
  RCLCPP_DEBUG(get_logger(), "=======================================");
  RCLCPP_DEBUG(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_DEBUG(get_logger(), " * node name: %s", get_name());
  RCLCPP_DEBUG(
    get_logger(), " * IPC: %s",
    options.use_intra_process_comms() ? "enabled" : "disabled");
  RCLCPP_DEBUG(get_logger(), "=======================================");

  // Read the parameters from the parameter server
  read_parameters();

  // Initialize the benchmark results statistics
  initialize_benchmark_results();

  // Create the thread to periodically retrieve CPU and GPU load
  _cpuGpuLoadThread = std::thread(&ZedNitrosMultiSubComponent::cpu_gpu_load_callback, this);

  // Start the example
  if (!_disableDDSTest) {
    // Create and enable the standard ROS 2 subscriber
    create_dds_subscriber();
  } else {
    // Create and enable the Nitros subscriber
    create_nitros_subscriber();
  }
}

void ZedNitrosMultiSubComponent::initialize_benchmark_results()
{
  // Initialize the reference time
  _lastMsgTime = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Initialize the benchmark results statistics with a lambda function
  auto init_stat = [](BenchmarkTest & stat, const std::string & name, const std::string & units) {
      stat.name = name;
      stat.units = units;
      stat.values.clear();
      stat.sum = 0.0;
      stat.min_val = std::numeric_limits<double>::max();
      stat.max_val = 0.0;
      stat.avg_val = 0.0;
      stat.std_dev_val = 0.0;
    };

  // Initialize both DDS and Nitros benchmark results
  _benchmarkResults.resize(_numCams);
  for(int i = 0; i < _numCams; ++i) {
    BenchmarkResults & benchmarkResults = _benchmarkResults[i];
    std::string test_prefix = _camNames[i] + " - ";
    init_stat(benchmarkResults.latency_dds, test_prefix + "DDS Subscriber Latency", "sec");
    init_stat(benchmarkResults.latency_nitros, test_prefix + "Nitros Subscriber Latency", "sec");
    init_stat(benchmarkResults.sub_freq_dds, test_prefix + "DDS Subscriber Frequency", "Hz");
    init_stat(benchmarkResults.sub_freq_nitros, test_prefix + "Nitros Subscriber Frequency", "Hz");
    init_stat(benchmarkResults.cpu_load_dds, test_prefix + "DDS Subscriber CPU Load", "%");
    init_stat(benchmarkResults.cpu_load_nitros, test_prefix + "Nitros Subscriber CPU Load", "%");
    init_stat(benchmarkResults.gpu_load_dds, test_prefix + "DDS Subscriber GPU Load", "%");
    init_stat(benchmarkResults.gpu_load_nitros, test_prefix + "Nitros Subscriber GPU Load", "%");
  }
}

void ZedNitrosMultiSubComponent::update_benchmark_stats(
  BenchmarkTest & benchmark, double new_value)
{
  benchmark.values.push_back(new_value);
  benchmark.sum += new_value;
  RCLCPP_DEBUG(
    this->get_logger(),
    "   * New sample value: %.6f sec - Min: %.6f %s", new_value,
    benchmark.min_val, benchmark.units.c_str());
  if (new_value < benchmark.min_val) {
    benchmark.min_val = new_value;
    RCLCPP_DEBUG(
      this->get_logger(), " * New min value: %.6f %s",
      benchmark.min_val, benchmark.units.c_str());
  }
  RCLCPP_DEBUG(
    this->get_logger(),
    "   * New sample value: %.6f sec - Max: %.6f %s", new_value,
    benchmark.max_val, benchmark.units.c_str());
  if (new_value > benchmark.max_val) {
    benchmark.max_val = new_value;
    RCLCPP_DEBUG(
      this->get_logger(), " * New max value: %.6f %s",
      benchmark.max_val, benchmark.units.c_str());
  }
}

void ZedNitrosMultiSubComponent::calculate_benchmark_results(BenchmarkTest & benchmark)
{
  if (benchmark.values.empty()) {
    RCLCPP_WARN(this->get_logger(), "No samples received, cannot calculate benchmark results.");
    return;
  }

  double sample_count = static_cast<double>(benchmark.values.size());

  benchmark.avg_val = benchmark.sum / sample_count;

  double sum_sq_diff = 0.0;
  for (const auto & sample_val : benchmark.values) {
    double diff = sample_val - benchmark.avg_val;
    sum_sq_diff += diff * diff;
  }
  benchmark.std_dev_val = std::sqrt(sum_sq_diff / sample_count);

  RCLCPP_DEBUG(this->get_logger(), " - %s: ", benchmark.name.c_str());
  RCLCPP_DEBUG(this->get_logger(), "   * Samples: %lu", static_cast<unsigned long>(sample_count));
  RCLCPP_DEBUG(this->get_logger(), "   * Min: %.6f %s", benchmark.min_val, benchmark.units.c_str());
  RCLCPP_DEBUG(this->get_logger(), "   * Max: %.6f %s", benchmark.max_val, benchmark.units.c_str());
  RCLCPP_DEBUG(this->get_logger(), "   * Avg: %.6f %s", benchmark.avg_val, benchmark.units.c_str());
  RCLCPP_DEBUG(
    this->get_logger(), " * Std Dev: %.6f %s",
    benchmark.std_dev_val, benchmark.units.c_str());

  if (!_csvLogFile.empty()) {
    std::ofstream csv_file;
    csv_file.open(_csvLogFile, std::ofstream::out | std::ofstream::app);
    if (!csv_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV log file: %s", _csvLogFile.c_str());
      return;
    }
    csv_file << benchmark.name << ", "
             << sample_count << ", "
             << benchmark.units << ", "
             << std::setprecision(6)
             << benchmark.avg_val << ", "
             << benchmark.min_val << ", "
             << benchmark.max_val << ", "
             << benchmark.std_dev_val << std::endl;
    csv_file.close();
  }
}

void ZedNitrosMultiSubComponent::compare_benchmark_results(
  const BenchmarkTest & benchmark1,
  const BenchmarkTest & benchmark2)
{
  std::ofstream csv_file;
  if (!_csvLogFile.empty()) {
    csv_file.open(_csvLogFile, std::ofstream::out | std::ofstream::app);
    csv_file << std::endl;
  }

  std::stringstream ss;
  RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
  if (csv_file.is_open()) {csv_file << ss.str() << std::endl;}

  ss.str(std::string());
  ss << " * " << benchmark1.name << " vs " << benchmark2.name;
  RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
  if (csv_file.is_open()) {csv_file << ss.str() << std::endl;}

  // Compare averages
  ss.str(std::string());
  ss << std::setprecision(6) << "   * Avg: " << benchmark1.avg_val << " " <<
    benchmark1.units.c_str() << " vs " << benchmark2.avg_val << " " << benchmark2.units.c_str();
  RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
  if (csv_file.is_open()) {csv_file << ss.str() << std::endl;}

  if (benchmark1.avg_val > benchmark2.avg_val) {
    double diff = benchmark1.avg_val - benchmark2.avg_val;
    double percent = (diff / benchmark2.avg_val) * 100.0;
    ss.str(std::string());
    ss << std::setprecision(6) << "     - " << benchmark1.name.c_str() << " is higher by " <<
      diff << " " << benchmark1.units.c_str() << " (" << percent << "%)";
    RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
    if (csv_file.is_open()) {csv_file << ss.str() << std::endl;}
  } else if (benchmark2.avg_val > benchmark1.avg_val) {
    double diff = benchmark2.avg_val - benchmark1.avg_val;
    double percent = (diff / benchmark1.avg_val) * 100.0;
    ss.str(std::string());
    ss << std::setprecision(6) << "     - " << benchmark2.name.c_str() << " is higher by " <<
      diff << " " << benchmark2.units.c_str() << " (" << percent << "%)";
    RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
    if (csv_file.is_open()) {csv_file << ss.str() << std::endl;}
  } else {
    ss.str(std::string());
    ss << "     - Both have the same average.";
    RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
    if (csv_file.is_open()) {csv_file << ss.str() << std::endl;}
  }

  if (csv_file.is_open()) {csv_file.close();}
}

void ZedNitrosMultiSubComponent::read_parameters()
{
  RCLCPP_DEBUG(get_logger(), "--------------------");
  RCLCPP_DEBUG(get_logger(), " Example Parameters");
  RCLCPP_DEBUG(get_logger(), "--------------------");

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;

  std::string paramName = "";

  // Number of samples to process before exiting
  descriptor.description = "Total number of samples to process before exiting";
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = 1;
  range.to_value = 1000000;
  descriptor.integer_range.push_back(range);
  paramName = "benchmark.tot_samples";
  this->declare_parameter(paramName, rclcpp::ParameterValue(_totSamples), descriptor);
  if (!this->get_parameter(paramName, _totSamples)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << _totSamples);
  }
  RCLCPP_DEBUG_STREAM(get_logger(), " * " << paramName << ": " << _totSamples);

  // Camera names
  descriptor.description = "Names of the cameras to subscribe to";
  paramName = "cameras.cam_names";
  this->declare_parameter(paramName, rclcpp::ParameterValue(_camNames), descriptor);
  if (!this->get_parameter(paramName, _camNames)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid. Stopping the node.  ");
    rclcpp::shutdown();
    return;
  }
  std::stringstream ss;
  for (const auto & cam_name : _camNames) {
    ss << cam_name << " ";
  }
  RCLCPP_DEBUG_STREAM(get_logger(), " * " << paramName << ": " << ss.str());

  _numCams = static_cast<int>(_camNames.size());

  // CPU and GPU load retrieval period
  descriptor.description = "Period in milliseconds to retrieve CPU and GPU load statistics";
  range.from_value = 10;
  range.to_value = 1000;
  descriptor.integer_range.clear(); // Clear previous range before reusing
  descriptor.integer_range.push_back(range);
  paramName = "benchmark.cpu_gpu_load_period";
  this->declare_parameter(paramName, rclcpp::ParameterValue(_cpuGpuLoadPeriod), descriptor);
  if (!this->get_parameter(paramName, _cpuGpuLoadPeriod)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << _cpuGpuLoadPeriod);
  }
  RCLCPP_DEBUG_STREAM(get_logger(), " * " << paramName << ": " << _cpuGpuLoadPeriod);

  // Size of the averaging window for CPU and GPU load statistics
  descriptor.description = "Size of the averaging window for CPU and GPU load statistics";
  range.from_value = 1;
  range.to_value = 100;
  descriptor.integer_range.clear(); // Clear previous range before reusing
  descriptor.integer_range.push_back(range);
  paramName = "benchmark.cpu_gpu_load_avg_wnd_size";
  this->declare_parameter(paramName, rclcpp::ParameterValue(_cpuGpuLoadAvgWndSize), descriptor);
  if (!this->get_parameter(paramName, _cpuGpuLoadAvgWndSize)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << _cpuGpuLoadAvgWndSize);
  }
  RCLCPP_DEBUG_STREAM(get_logger(), " * " << paramName << ": " << _cpuGpuLoadAvgWndSize);

  // Disable DDS test
  descriptor.description = "Disable the DDS benchmark test";
  paramName = "benchmark.disable_dds_test";
  this->declare_parameter(paramName, rclcpp::ParameterValue(_disableDDSTest), descriptor);
  if (!this->get_parameter(paramName, _disableDDSTest)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << _disableDDSTest);
  }
  RCLCPP_DEBUG_STREAM(
    get_logger(), " * " << paramName << ": " << (_disableDDSTest ? "TRUE" : "FALSE"));

  // CSV log file
  descriptor.description =
    "If not empty, log the benchmark results in a CSV file with the specified name";
  paramName = "benchmark.csv_log_file";
  this->declare_parameter(paramName, rclcpp::ParameterValue(_csvLogFile), descriptor);
  if (!this->get_parameter(paramName, _csvLogFile)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << _csvLogFile);
  }
  RCLCPP_DEBUG_STREAM(get_logger(), " * " << paramName << ": " << _csvLogFile);

  // Isaac ROS Nitros debug information
  descriptor.description = "Isaac ROS Nitros debug information";
  paramName = "debug.debug_nitros";
  this->declare_parameter(paramName, rclcpp::ParameterValue(_debugNitros), descriptor);
  if (!this->get_parameter(paramName, _debugNitros)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << _totSamples);
  }
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    " * " << paramName << ": " << (_debugNitros ? "TRUE" : "FALSE"));
  if (_debugNitros) {
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);
  }
}

void ZedNitrosMultiSubComponent::create_dds_subscribers()
{
  // Create subscribers to the image topics
  for (int i = 0; i < _numCams; ++i) {
    std::string topic_name = _camNames[i] + "/image";
    auto sub = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name, _defaultQoS,
      std::bind(
        &ZedNitrosMultiSubComponent::dds_sub_callback, this, 
        std::placeholders::_1, i));
    _subArray.push_back(sub);
  }
} 

void ZedNitrosMultiSubComponent::create_nitros_subscribers()
{
  // Create a Nitros subscribers to the Nitros topics
  for (int i = 0; i < _numCams; ++i) {
    std::string topic_name = _camNames[i] +   "image/nitros";
    _nitrosSubArray.push_back(nitrosSub);
}

void ZedNitrosMultiSubComponent::dds_sub_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img, int source_index) {
  auto now = this->now();

  static bool first_frame = true;
  if (first_frame) {
    _encoding = img->encoding;
    first_frame = false;
    if (img->encoding == "32FC1") {

      _isDepth = true;
      RCLCPP_DEBUG(this->get_logger(), "Receiving Depth map messages.");
    } else if (img->encoding == "bgra8") {
      _isDepth = false;
      RCLCPP_DEBUG(this->get_logger(), "Receiving RGB image messages.");
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Received an unsupported image encoding: %s. Only BGRA and 32FC1 are supported.",
        img->encoding.c_str());
      rclcpp::shutdown();
    }
  }

  // Check example coherency
  if (img->encoding != _encoding) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Topic type has changed from %s to %s. This is not supported. Exiting.",
      _encoding.c_str(), img->encoding.c_str());
    rclcpp::shutdown();
  }

  RCLCPP_DEBUG(this->get_logger(), "Receiving Image buffer with memory at: %p", img->data.data());
  RCLCPP_DEBUG(
    this->get_logger(), " * encoding: %s", img->encoding.c_str());
  RCLCPP_DEBUG(
    this->get_logger(), " * width: %d, height: %d", img->width, img->height);

  // ----> FPS statistics update
  if (_lastMsgTime.nanoseconds() != 0) {
    double time_diff = (now - _lastMsgTime).seconds();
    double fps = 1.0 / time_diff;
    RCLCPP_DEBUG(this->get_logger(), " * FPS: %f", fps);
    // Update benchmark results here
    update_benchmark_stats(_benchmarkResults.sub_freq_dds, fps);
  }
  _lastMsgTime = now;
  // <---- FPS statistics update

  // ----> Latency statistics update
  rclcpp::Time img_ts;
  int32_t ts_sec = img->header.stamp.sec;
  int32_t ts_nsec = img->header.stamp.nanosec;
  img_ts = rclcpp::Time(ts_sec, ts_nsec, now.get_clock_type());

  double latency = now.seconds() - img_ts.seconds();

  RCLCPP_DEBUG(
    this->get_logger(), " * Latency: %f sec", latency);

  // Update benchmark results here
  update_benchmark_stats(_benchmarkResults.latency_dds, latency);
  // <---- Latency statistics update

  // ----> CPU and GPU load statistics update
  double cpu_load = _cpuLoadAvg.load();
  double gpu_load = _gpuLoadAvg.load();
  RCLCPP_DEBUG(
    this->get_logger(), " * CPU Load: %.2f%%", cpu_load);
  update_benchmark_stats(_benchmarkResults.cpu_load_dds, cpu_load);
  RCLCPP_DEBUG(
    this->get_logger(), " * GPU Load: %.2f%%", gpu_load);
  update_benchmark_stats(_benchmarkResults.gpu_load_dds, gpu_load);
  // <---- CPU and GPU load statistics update

  // Check if we acquired the desired number of samples
  static int std_sample_count = 0;
  std_sample_count++; \
  RCLCPP_DEBUG(
    this->get_logger(), " *** Sample %d/%d ***", std_sample_count, _totSamples);
  if (std_sample_count >= _totSamples) {
    RCLCPP_DEBUG(
      this->get_logger(), "Received %d DDS samples. Unsubscribing...",
      _totSamples);
    _sub.reset();
    RCLCPP_DEBUG(this->get_logger(), "DDS subscriber unsubscribed.");
    RCLCPP_DEBUG(this->get_logger(), "-----------------------------------");

    // Reset the reference time
    _lastMsgTime = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // Create and enable the Nitros subscriber
    create_nitros_subscriber();
  }
}

void ZedNitrosMultiSubComponent::nitros_sub_callback(
    const nvidia::isaac_ros::nitros::NitrosImageView& img, int source_index) {
  // Check example coherency
  if (!_disableDDSTest) {
    if (img.GetEncoding() != _encoding) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Topic type has changed from %s to %s. This is not "
        "supported. Exiting.",
        _encoding.c_str(), img.GetEncoding().c_str());
      rclcpp::shutdown();
    }
  }

  auto now = this->now();

  RCLCPP_DEBUG(
    this->get_logger(), "Receiving CUDA buffer with memory at: %p",
    img.GetGpuData());
  RCLCPP_DEBUG(
    this->get_logger(), " * encoding: %s",
    img.GetEncoding().c_str());
  RCLCPP_DEBUG(
    this->get_logger(), " * width: %d, height: %d", img.GetWidth(),
    img.GetHeight());

  // ----> FPS statistics update
  if (_lastMsgTime.nanoseconds() != 0) {
    double time_diff = (now - _lastMsgTime).seconds();
    double fps = 1.0 / time_diff;
    RCLCPP_DEBUG(this->get_logger(), " * FPS: %f", fps);
    // Update benchmark results here
    update_benchmark_stats(_benchmarkResults.sub_freq_nitros, fps);
  }
  _lastMsgTime = now;
  // <---- FPS statistics update

  // ----> Latency statistics update
  rclcpp::Time img_ts;
  int32_t ts_sec = img.GetTimestampSeconds();
  int32_t ts_nsec = img.GetTimestampNanoseconds();
  img_ts = rclcpp::Time(ts_sec, ts_nsec, now.get_clock_type());
  double latency = now.seconds() - img_ts.seconds();
  RCLCPP_DEBUG(this->get_logger(), " * Latency: %f sec", latency);

  // Update benchmark results here
  update_benchmark_stats(_benchmarkResults.latency_nitros, latency);
  // <---- Latency statistics update

  // ----> CPU and GPU load statistics update
  double cpu_load = _cpuLoadAvg.load();
  double gpu_load = _gpuLoadAvg.load();
  RCLCPP_DEBUG(this->get_logger(), " * CPU Load: %.2f%%", cpu_load);
  update_benchmark_stats(_benchmarkResults.cpu_load_nitros, cpu_load);
  RCLCPP_DEBUG(this->get_logger(), " * GPU Load: %.2f%%", gpu_load);
  update_benchmark_stats(_benchmarkResults.gpu_load_nitros, gpu_load);
  // <---- CPU and GPU load statistics update

  // Check if we acquired the desired number of samples
  static int nitros_sample_count = 0;
  nitros_sample_count++;
  RCLCPP_DEBUG(
    this->get_logger(), " *** Sample %d/%d ***",
    nitros_sample_count, _totSamples);
  if (nitros_sample_count >= _totSamples) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Received %d Nitros samples. Unsubscribing...", _totSamples);
    _nitrosSub.reset();
    RCLCPP_DEBUG(this->get_logger(), "Nitros subscriber unsubscribed.");

    // Calculate the final statistics
    print_benchmark_results();

    // Perform a clean shutdown of the node
    RCLCPP_DEBUG(this->get_logger(), "Shutting down the node...");
    _cpuGpuLoadThreadRunning = false;
    if (_cpuGpuLoadThread.joinable()) {
      _cpuGpuLoadThread.join();
    }
    rclcpp::shutdown();
  }
}

void ZedNitrosMultiSubComponent::print_benchmark_results()
{
  if (!_csvLogFile.empty()) {
    std::ofstream csv_file;
    csv_file.open(_csvLogFile, std::ofstream::out | std::ofstream::trunc);
    if (!csv_file.is_open()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Could not open CSV file: %s for writing benchmark results.",
        _csvLogFile.c_str());

      _csvLogFile = "";
    } else {
      RCLCPP_DEBUG(
        this->get_logger(),
        "+++ Logging benchmark results in CSV file: %s +++",
        _csvLogFile.c_str());

      csv_file << "BENCHMARK, SAMPLES, UNITS, AVG, MIN, MAX, STDDEV\n";
      csv_file.close();
    }
  }

  if (!_disableDDSTest) {
    RCLCPP_DEBUG(this->get_logger(), "-----------------------------------");
    RCLCPP_DEBUG(this->get_logger(), "DDS Subscriber benchmark results:");
    calculate_benchmark_results(_benchmarkResults.sub_freq_dds);
    calculate_benchmark_results(_benchmarkResults.latency_dds);
    calculate_benchmark_results(_benchmarkResults.cpu_load_dds);
    calculate_benchmark_results(_benchmarkResults.gpu_load_dds);
  }
  RCLCPP_DEBUG(this->get_logger(), "-----------------------------------");
  RCLCPP_DEBUG(this->get_logger(), "Nitros Subscriber benchmark results:");
  calculate_benchmark_results(_benchmarkResults.sub_freq_nitros);
  calculate_benchmark_results(_benchmarkResults.latency_nitros);
  calculate_benchmark_results(_benchmarkResults.cpu_load_nitros);
  calculate_benchmark_results(_benchmarkResults.gpu_load_nitros);
  RCLCPP_DEBUG(this->get_logger(), "-----------------------------------");

  if (!_disableDDSTest) {
    RCLCPP_DEBUG(this->get_logger(), "Benchmark comparison:");
    compare_benchmark_results(
      _benchmarkResults.latency_dds,
      _benchmarkResults.latency_nitros);
    compare_benchmark_results(
      _benchmarkResults.sub_freq_dds,
      _benchmarkResults.sub_freq_nitros);
    compare_benchmark_results(
      _benchmarkResults.cpu_load_dds,
      _benchmarkResults.cpu_load_nitros);
    compare_benchmark_results(
      _benchmarkResults.gpu_load_dds,
      _benchmarkResults.gpu_load_nitros);
    RCLCPP_DEBUG(this->get_logger(), "-----------------------------------");
  }
}

double ZedNitrosMultiSubComponent::get_cpu_load()
{
  std::ifstream file("/proc/stat");
  std::string line;
  std::getline(file, line);

  std::istringstream iss(line);
  std::string cpu;
  unsigned long user, nice, system, idle;
  iss >> cpu >> user >> nice >> system >> idle;

  static unsigned long prevTotal = 0, prevIdle = 0;
  unsigned long total = user + nice + system + idle;

  double diffTotal = total - prevTotal;
  double diffIdle = idle - prevIdle;

  prevTotal = total;
  prevIdle = idle;

  if (diffTotal == 0) {return 0.0f;}
  return 100.0f * (1.0f - diffIdle / diffTotal);
}

double ZedNitrosMultiSubComponent::get_gpu_load()
{
  std::ifstream file("/sys/devices/platform/gpu.0/load");
  int value = 0;
  file >> value;
  file.close();

  // The value represents the percentage * 10 (e.g., 450 = 45%)
  return value / 10.0f;
}

void ZedNitrosMultiSubComponent::cpu_gpu_load_callback()
{
  static int count = 0;
  while (_cpuGpuLoadThreadRunning && rclcpp::ok()) {
    double cpu_load = get_cpu_load();
    double gpu_load = get_gpu_load();

    // ----> Simple moving average
    static std::deque<double> cpu_queue, gpu_queue;

    cpu_queue.push_back(cpu_load);
    gpu_queue.push_back(gpu_load);

    if (cpu_queue.size() > _cpuGpuLoadAvgWndSize) {cpu_queue.pop_front();}
    if (gpu_queue.size() > _cpuGpuLoadAvgWndSize) {gpu_queue.pop_front();}

    double cpu_avg = std::accumulate(cpu_queue.begin(), cpu_queue.end(), 0.0f) / cpu_queue.size();
    double gpu_avg = std::accumulate(gpu_queue.begin(), gpu_queue.end(), 0.0f) / gpu_queue.size();
    // <---- Simple moving average

    //RCLCPP_DEBUG(this->get_logger(), "CPU avg (last 5): %.2f%%, GPU avg (last 5): %.2f%%", cpu_avg, gpu_avg);
    //RCLCPP_DEBUG(this->get_logger(), "#%d - CPU Load: %.2f%% - GPU Load: %.2f%%", count++, cpu_load, gpu_load);
    _cpuLoadAvg = cpu_avg;
    _gpuLoadAvg = gpu_avg;

    std::this_thread::sleep_for(std::chrono::milliseconds(_cpuGpuLoadPeriod));
  }
}

}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedNitrosMultiSubComponent)
