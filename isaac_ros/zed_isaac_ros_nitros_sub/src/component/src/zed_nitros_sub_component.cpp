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

#include "zed_nitros_sub_component.hpp"

namespace stereolabs
{

ZedNitrosSubComponent::ZedNitrosSubComponent(const rclcpp::NodeOptions & options)
: Node("zed_nitros_sub_component", options), _defaultQoS(10)
{
  RCLCPP_INFO(get_logger(), "=================================");
  RCLCPP_INFO(get_logger(), " ZED Nitros Subscriber Component");
  RCLCPP_INFO(get_logger(), "=================================");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(
    get_logger(), " * IPC: %s",
    options.use_intra_process_comms() ? "enabled" : "disabled");
  RCLCPP_INFO(get_logger(), "================================");

  // Read the parameters from the parameter server
  read_parameters();

  // Initialize the benchmark results statistics
  initialize_benchmark_results();

  // Start the example and perform the benchmark
  create_dds_subscriber();
}

void ZedNitrosSubComponent::initialize_benchmark_results()
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
  init_stat(_benchmarkResults.latency_dds, "DDS Subscriber Latency", "sec");
  init_stat(_benchmarkResults.latency_nitros, "Nitros Subscriber Latency", "sec");
  init_stat(_benchmarkResults.sub_freq_dds, "DDS Subscriber Frequency", "Hz");
  init_stat(_benchmarkResults.sub_freq_nitros, "Nitros Subscriber Frequency", "Hz");
}

void ZedNitrosSubComponent::update_benchmark_stats(
  BenchmarkTest & benchmark, double new_value)
{
  benchmark.values.push_back(new_value);
  benchmark.sum += new_value;
  RCLCPP_INFO(
    this->get_logger(),
    " * New sample value: %.6f sec - Min: %.6f %s", new_value,
    benchmark.min_val, benchmark.units.c_str());
  if (new_value < benchmark.min_val) {
    benchmark.min_val = new_value;
    RCLCPP_INFO(
      this->get_logger(), " * New min value: %.6f %s",
      benchmark.min_val, benchmark.units.c_str());
  }
  RCLCPP_INFO(
    this->get_logger(),
    " * New sample value: %.6f sec - Max: %.6f %s", new_value,
    benchmark.max_val, benchmark.units.c_str());
  if (new_value > benchmark.max_val) {
    benchmark.max_val = new_value;
    RCLCPP_INFO(
      this->get_logger(), " * New max value: %.6f %s",
      benchmark.max_val, benchmark.units.c_str());
  }
}

void ZedNitrosSubComponent::calculate_benchmark_results(BenchmarkTest & benchmark)
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

  RCLCPP_INFO(this->get_logger(), " - Benchmark results for '%s': ", benchmark.name.c_str());
  RCLCPP_INFO(this->get_logger(), "   * Samples: %lu", static_cast<unsigned long>(sample_count));
  RCLCPP_INFO(this->get_logger(), "   * Min: %.6f %s", benchmark.min_val, benchmark.units.c_str());
  RCLCPP_INFO(this->get_logger(), "   * Max: %.6f %s", benchmark.max_val, benchmark.units.c_str());
  RCLCPP_INFO(this->get_logger(), "   * Avg: %.6f %s", benchmark.avg_val, benchmark.units.c_str());
  RCLCPP_INFO(
    this->get_logger(), " * Std Dev: %.6f %s",
    benchmark.std_dev_val, benchmark.units.c_str());
}

void ZedNitrosSubComponent::compare_benchmark_results(
  const BenchmarkTest & benchmark1,
  const BenchmarkTest & benchmark2)
{
  RCLCPP_INFO(this->get_logger(), "Benchmark comparison:");
  RCLCPP_INFO(this->get_logger(), " * %s vs %s", benchmark1.name.c_str(), benchmark2.name.c_str());

  RCLCPP_INFO(
    this->get_logger(), "   * Avg Frequency: %.2f Hz vs %.2f Hz",
    benchmark1.avg_val, benchmark2.avg_val);
  if (benchmark1.avg_val > benchmark2.avg_val) {
    double diff = benchmark1.avg_val - benchmark2.avg_val;
    double percent = (diff / benchmark2.avg_val) * 100.0;
    RCLCPP_INFO(
      this->get_logger(), "     - %s is higher by %.2f Hz (%.2f%% faster)",
      benchmark1.name.c_str(), diff, percent);
  } else if (benchmark2.avg_val > benchmark1.avg_val) {
    double diff = benchmark2.avg_val - benchmark1.avg_val;
    double percent = (diff / benchmark1.avg_val) * 100.0;
    RCLCPP_INFO(
      this->get_logger(), "     - %s is higher by %.2f Hz (%.2f%% faster)",
      benchmark2.name.c_str(), diff, percent);
  } else {
    RCLCPP_INFO(this->get_logger(), "     - Both have the same average frequency.");
  }  

  RCLCPP_INFO(
    this->get_logger(), "   * Avg Latency: %.6f sec vs %.6f sec",
    benchmark1.avg_val, benchmark2.avg_val);
  if (benchmark1.avg_val < benchmark2.avg_val) {
    double diff = benchmark2.avg_val - benchmark1.avg_val;
    double percent = (diff / benchmark1.avg_val) * 100.0;
    RCLCPP_INFO(
      this->get_logger(), "     - %s is lower by %.6f sec (%.2f%% faster)",
      benchmark1.name.c_str(), diff, percent);
  } else if (benchmark2.avg_val < benchmark1.avg_val) {
    double diff = benchmark1.avg_val - benchmark2.avg_val;
    double percent = (diff / benchmark2.avg_val) * 100.0;
    RCLCPP_INFO(
      this->get_logger(), "     - %s is lower by %.6f sec (%.2f%% faster)",
      benchmark2.name.c_str(), diff, percent);
  } else {
    RCLCPP_INFO(this->get_logger(), "     - Both have the same average latency.");
  }

   
}

void ZedNitrosSubComponent::read_parameters()
{
  RCLCPP_INFO(get_logger(), "--------------------");
  RCLCPP_INFO(get_logger(), " Example Parameters");
  RCLCPP_INFO(get_logger(), "--------------------");

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;

  std::string paramName = "";

  // Number of samples to process before exiting
  descriptor.description = "Total number of samples to process before exiting";
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = 1;
  range.to_value = 1000;
  descriptor.integer_range.push_back(range);
  paramName = "test.tot_samples";
  this->declare_parameter(paramName, rclcpp::ParameterValue(_totSamples), descriptor);
  if (!this->get_parameter(paramName, _totSamples)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << _totSamples);
  }
  RCLCPP_INFO_STREAM(get_logger(), " * " << paramName << ": " << _totSamples);

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
  RCLCPP_INFO_STREAM(get_logger(), " * " << paramName << ": " << (_debugNitros ? "TRUE" : "FALSE"));
  if (_debugNitros) {
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);
  }
}

void ZedNitrosSubComponent::create_dds_subscriber()
{
  // Create a subscriber to the image topic
  _sub = this->create_subscription<sensor_msgs::msg::Image>(
    "image", _defaultQoS,
    std::bind(
      &ZedNitrosSubComponent::dds_sub_callback, this,
      std::placeholders::_1));
}

void ZedNitrosSubComponent::create_nitros_subscriber()
{
  // Create a Nitros subscriber to the Nitros topic
  _nitrosSub = std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
        nvidia::isaac_ros::nitros::NitrosImageView>>(
    this, "image",
    _isDepth ? nvidia::isaac_ros::nitros::nitros_image_32FC1_t::supported_type_name :
    nvidia::isaac_ros::nitros::nitros_image_rgba8_t::supported_type_name,
    std::bind(
      &ZedNitrosSubComponent::nitros_sub_callback, this,
      std::placeholders::_1));
}

void ZedNitrosSubComponent::dds_sub_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & img)
{
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

  RCLCPP_INFO(this->get_logger(), "Receiving Image buffer with memory at: %p", img->data.data());
  RCLCPP_INFO(
    this->get_logger(), " * encoding: %s", img->encoding.c_str());
  RCLCPP_INFO(
    this->get_logger(), " * width: %d, height: %d", img->width, img->height);

  // ----> FPS statistics update
  if (_lastMsgTime.nanoseconds() != 0) {
    double time_diff = (now - _lastMsgTime).seconds();
    double fps = 1.0 / time_diff;
    RCLCPP_INFO(this->get_logger(), " * FPS: %f", fps);
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

  RCLCPP_INFO(
    this->get_logger(), " * Latency: %f sec", latency);

  // Check if we acquired the desired number of samples
  static int std_sample_count = 0;
  std_sample_count++; \
  RCLCPP_INFO(
    this->get_logger(), " *** Sample %d/%d ***", std_sample_count, _totSamples);

  // Update benchmark results here
  update_benchmark_stats(_benchmarkResults.latency_dds, latency);
  // <---- Latency statistics update

  // Check if we acquired the desired number of samples
  if (std_sample_count >= _totSamples) {
    RCLCPP_INFO(
      this->get_logger(), "Received %d DDS samples. Unsubscribing...",
      _totSamples);
    _sub.reset();
    RCLCPP_INFO(this->get_logger(), "DDS subscriber unsubscribed.");
    RCLCPP_INFO(this->get_logger(), "-----------------------------------");

    // Reset the reference time
    _lastMsgTime = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // Create and enable the Nitros subscriber
    create_nitros_subscriber();
  }
}

void ZedNitrosSubComponent::nitros_sub_callback(
  const nvidia::isaac_ros::nitros::NitrosImageView & img)
{
  // Check example coherency
  if (img.GetEncoding() != _encoding) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Topic type has changed from %s to %s. This is not supported. Exiting.",
      _encoding.c_str(), img.GetEncoding().c_str());
    rclcpp::shutdown();
  }

  auto now = this->now();

  RCLCPP_INFO(this->get_logger(), "Receiving CUDA buffer with memory at: %p", img.GetGpuData());
  RCLCPP_INFO(
    this->get_logger(),
    " * encoding: %s", img.GetEncoding().c_str());
  RCLCPP_INFO(
    this->get_logger(),
    " * width: %d, height: %d",
    img.GetWidth(), img.GetHeight());

  // ----> FPS statistics update
  if (_lastMsgTime.nanoseconds() != 0) {
    double time_diff = (now - _lastMsgTime).seconds();
    double fps = 1.0 / time_diff;
    RCLCPP_INFO(this->get_logger(), " * FPS: %f", fps);
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
  RCLCPP_INFO(this->get_logger(), " * Latency: %f sec", latency);

  // Check if we acquired the desired number of samples
  static int nitros_sample_count = 0;
  nitros_sample_count++;
  RCLCPP_INFO(
    this->get_logger(), " *** Sample %d/%d ***", nitros_sample_count, _totSamples);

  // Update benchmark results here
  update_benchmark_stats(_benchmarkResults.latency_nitros, latency);
  // <---- Latency statistics update

  // Check if we acquired the desired number of samples
  if (nitros_sample_count >= _totSamples) {
    RCLCPP_INFO(this->get_logger(), "Received %d Nitros samples. Unsubscribing...", _totSamples);
    _nitrosSub.reset();
    RCLCPP_INFO(this->get_logger(), "Nitros subscriber unsubscribed.");

    // Calculate the statistics
    RCLCPP_INFO(this->get_logger(), "-----------------------------------");
    RCLCPP_INFO(this->get_logger(), "DDS Subscriber benchmark results:");
    calculate_benchmark_results(_benchmarkResults.sub_freq_dds);
    calculate_benchmark_results(_benchmarkResults.latency_dds);
    RCLCPP_INFO(this->get_logger(), "-----------------------------------");
    RCLCPP_INFO(this->get_logger(), "Nitros Subscriber benchmark results:");
    calculate_benchmark_results(_benchmarkResults.sub_freq_nitros);
    calculate_benchmark_results(_benchmarkResults.latency_nitros);
    RCLCPP_INFO(this->get_logger(), "-----------------------------------");
    compare_benchmark_results(
      _benchmarkResults.latency_dds,
      _benchmarkResults.latency_nitros);
    RCLCPP_INFO(this->get_logger(), "-----------------------------------");

    // Perform a clean shutdown of the node
    RCLCPP_INFO(this->get_logger(), "Shutting down the node...");
    rclcpp::shutdown();
  }
}

float ZedNitrosSubComponent::get_cpu_load()
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

  float diffTotal = total - prevTotal;
  float diffIdle = idle - prevIdle;

  prevTotal = total;
  prevIdle = idle;

  if (diffTotal == 0) return 0.0f;
  return 100.0f * (1.0f - diffIdle / diffTotal);
}

float ZedNitrosSubComponent::get_gpu_load()
{
  std::string cmd = "tegrastats --interval 100 --count 1";
  std::array<char, 128> buffer;
  std::string result;
  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe) return -1;
  while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
      result += buffer.data();
  }
  pclose(pipe);

  // Esempio di parsing basato su output tipo: "GR3D_FREQ 45%@306MHz"
  size_t pos = result.find("GR3D_FREQ");
  if (pos != std::string::npos) {
      size_t pct_pos = result.find('%', pos);
      size_t at_pos = result.find('@', pos);
      if (pct_pos != std::string::npos && at_pos != std::string::npos) {
          std::string pct = result.substr(at_pos + 1, pct_pos - at_pos - 1);
          return std::stof(result.substr(pos + 10, pct_pos - pos - 10));
      }
  }
  return -1.0f;
}

}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedNitrosSubComponent)
