// Copyright 2026 Stereolabs
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

#include "zed_cuda_pc_sub_component.hpp"

#include <dlfcn.h>

#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>

// User-facing API of the CUDA buffer backend: `from_input_buffer` gives
// read-only access to the GPU memory of a CUDA-backed `rosidl::Buffer`
#include <cuda_buffer/cuda_buffer_api.hpp>

namespace stereolabs
{

namespace
{
// Minimal NVML interface, loaded at runtime to read the GPU load on the
// platforms that do not expose it in sysfs (Jetson Thor, x86_64)
struct NvmlUtilization
{
  unsigned int gpu;
  unsigned int memory;
};
using NvmlInitFn = int (*)();
using NvmlGetHandleFn = int (*)(unsigned int, void **);
using NvmlGetUtilizationFn = int (*)(void *, NvmlUtilization *);
}  // namespace

ZedCudaPcSubComponent::ZedCudaPcSubComponent(const rclcpp::NodeOptions & options)
: Node("zed_cuda_pc_sub_component", options), _defaultQoS(10)
{
  RCLCPP_INFO(get_logger(), "======================================");
  RCLCPP_INFO(get_logger(), " ZED CUDA Zero-Copy Point Cloud Subscriber");
  RCLCPP_INFO(get_logger(), "======================================");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(
    get_logger(), " * IPC: %s",
    options.use_intra_process_comms() ? "enabled" : "disabled");
  if (options.use_intra_process_comms()) {
    RCLCPP_WARN(
      get_logger(),
      "The ZED node publishes CPU data to the intra-process subscribers. "
      "Disable IPC for this component to receive CUDA buffers.");
  }
  RCLCPP_INFO(get_logger(), "======================================");

  // Read the parameters from the parameter server
  read_parameters();

  // CUDA stream used to read the received CUDA buffers
  cudaError_t err = cudaStreamCreateWithFlags(&_stream, cudaStreamNonBlocking);
  if (err != cudaSuccess) {
    RCLCPP_ERROR(
      get_logger(), "Cannot create the CUDA stream: %s", cudaGetErrorString(err));
    _stream = nullptr;
  }

  // Initialize the benchmark results statistics
  initialize_benchmark_results();

  // Create the thread to periodically retrieve CPU and GPU load
  _cpuGpuLoadThread = std::thread(&ZedCudaPcSubComponent::cpu_gpu_load_callback, this);

  // Start the example and perform the benchmark with standard CPU data
  create_subscriber(Phase::CPU);
}

ZedCudaPcSubComponent::~ZedCudaPcSubComponent()
{
  _cpuGpuLoadThreadRunning = false;
  if (_cpuGpuLoadThread.joinable()) {
    _cpuGpuLoadThread.join();
  }
  _sub.reset();
  if (_stream) {
    cudaStreamSynchronize(_stream);
    cudaStreamDestroy(_stream);
  }
  if (_devBuffer) {
    cudaFree(_devBuffer);
  }
}

void ZedCudaPcSubComponent::initialize_benchmark_results()
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

  init_stat(_cpuResults.latency, "CPU Subscriber Latency", "sec");
  init_stat(_cudaResults.latency, "CUDA Subscriber Latency", "sec");
  init_stat(_cpuResults.sub_period, "CPU Subscriber Period", "sec");
  init_stat(_cudaResults.sub_period, "CUDA Subscriber Period", "sec");
  init_stat(_cpuResults.cpu_load, "CPU Subscriber CPU Load", "%");
  init_stat(_cudaResults.cpu_load, "CUDA Subscriber CPU Load", "%");
  init_stat(_cpuResults.gpu_load, "CPU Subscriber GPU Load", "%");
  init_stat(_cudaResults.gpu_load, "CUDA Subscriber GPU Load", "%");
}

void ZedCudaPcSubComponent::update_benchmark_stats(
  BenchmarkTest & benchmark, double new_value)
{
  benchmark.values.push_back(new_value);
  benchmark.sum += new_value;
  benchmark.min_val = std::min(benchmark.min_val, new_value);
  benchmark.max_val = std::max(benchmark.max_val, new_value);
}

void ZedCudaPcSubComponent::calculate_benchmark_results(BenchmarkTest & benchmark)
{
  if (benchmark.values.empty()) {
    RCLCPP_WARN(
      this->get_logger(), "No samples received for '%s', cannot calculate the results.",
      benchmark.name.c_str());
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

  RCLCPP_INFO(this->get_logger(), " - %s: ", benchmark.name.c_str());
  RCLCPP_INFO(this->get_logger(), "   * Samples: %lu", static_cast<unsigned long>(sample_count));
  RCLCPP_INFO(this->get_logger(), "   * Min: %.6f %s", benchmark.min_val, benchmark.units.c_str());
  RCLCPP_INFO(this->get_logger(), "   * Max: %.6f %s", benchmark.max_val, benchmark.units.c_str());
  RCLCPP_INFO(this->get_logger(), "   * Avg: %.6f %s", benchmark.avg_val, benchmark.units.c_str());
  RCLCPP_INFO(
    this->get_logger(), "   * Std Dev: %.6f %s",
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

void ZedCudaPcSubComponent::compare_benchmark_results(
  const BenchmarkTest & benchmark1,
  const BenchmarkTest & benchmark2)
{
  std::ofstream csv_file;
  if (!_csvLogFile.empty()) {
    csv_file.open(_csvLogFile, std::ofstream::out | std::ofstream::app);
    csv_file << std::endl;
  }

  std::stringstream ss;
  ss << " * " << benchmark1.name << " vs " << benchmark2.name;
  RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  if (csv_file.is_open()) {csv_file << ss.str() << std::endl;}

  // Compare averages
  ss.str(std::string());
  ss << std::setprecision(6) << "   * Avg: " << benchmark1.avg_val << " " <<
    benchmark1.units << " vs " << benchmark2.avg_val << " " << benchmark2.units;
  RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  if (csv_file.is_open()) {csv_file << ss.str() << std::endl;}

  ss.str(std::string());
  if (benchmark1.avg_val > benchmark2.avg_val && benchmark2.avg_val != 0.0) {
    double diff = benchmark1.avg_val - benchmark2.avg_val;
    double percent = (diff / benchmark2.avg_val) * 100.0;
    ss << std::setprecision(6) << "     - " << benchmark1.name << " is higher by " <<
      diff << " " << benchmark1.units << " (" << percent << "%)";
  } else if (benchmark2.avg_val > benchmark1.avg_val && benchmark1.avg_val != 0.0) {
    double diff = benchmark2.avg_val - benchmark1.avg_val;
    double percent = (diff / benchmark1.avg_val) * 100.0;
    ss << std::setprecision(6) << "     - " << benchmark2.name << " is higher by " <<
      diff << " " << benchmark2.units << " (" << percent << "%)";
  } else {
    ss << "     - No relative difference available.";
  }
  RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  if (csv_file.is_open()) {csv_file << ss.str() << std::endl;}

  if (csv_file.is_open()) {csv_file.close();}
}

void ZedCudaPcSubComponent::read_parameters()
{
  RCLCPP_INFO(get_logger(), "--------------------");
  RCLCPP_INFO(get_logger(), " Example Parameters");
  RCLCPP_INFO(get_logger(), "--------------------");

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;

  // Lambda to declare and read an integer parameter with a range
  auto read_int_param =
    [&](const std::string & name, const std::string & description, int min, int max,
      int & value) {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.read_only = true;
      desc.description = description;
      rcl_interfaces::msg::IntegerRange range;
      range.from_value = min;
      range.to_value = max;
      desc.integer_range.push_back(range);
      this->declare_parameter(name, rclcpp::ParameterValue(value), desc);
      if (!this->get_parameter(name, value)) {
        RCLCPP_WARN_STREAM(
          get_logger(),
          "The parameter '" << name
                            << "' is not available or is not valid, using the default value: "
                            << value);
      }
      RCLCPP_INFO_STREAM(get_logger(), " * " << name << ": " << value);
    };

  read_int_param(
    "benchmark.tot_samples", "Number of samples to process for each benchmark phase",
    1, 1000, _totSamples);
  read_int_param(
    "benchmark.cpu_gpu_load_period",
    "Period in milliseconds to retrieve CPU and GPU load statistics", 10, 1000,
    _cpuGpuLoadPeriod);
  read_int_param(
    "benchmark.cpu_gpu_load_avg_wnd_size",
    "Size of the averaging window for CPU and GPU load statistics", 1, 100,
    _cpuGpuLoadAvgWndSize);

  // CSV log file
  std::string paramName = "benchmark.csv_log_file";
  descriptor.description =
    "If not empty, log the benchmark results in a CSV file with the specified name";
  this->declare_parameter(paramName, rclcpp::ParameterValue(_csvLogFile), descriptor);
  this->get_parameter(paramName, _csvLogFile);
  RCLCPP_INFO_STREAM(get_logger(), " * " << paramName << ": " << _csvLogFile);

  // Debug information
  paramName = "debug.debug_cuda";
  descriptor.description = "Enable the debug information of the example";
  this->declare_parameter(paramName, rclcpp::ParameterValue(_debugCuda), descriptor);
  this->get_parameter(paramName, _debugCuda);
  RCLCPP_INFO_STREAM(get_logger(), " * " << paramName << ": " << (_debugCuda ? "TRUE" : "FALSE"));
  if (_debugCuda) {
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);
  }
}

void ZedCudaPcSubComponent::create_subscriber(Phase phase)
{
  _phase = phase;
  _sampleCount = 0;
  _lastMsgTime = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // The subscription declares which buffer backends it accepts:
  // - "cpu": the default. The middleware always delivers CPU data, copying
  //   the GPU data of the ZED node to CPU memory when needed.
  // - "cuda": the middleware delivers the GPU memory of the ZED node without
  //   copies when the publisher is on the same host and GPU, and CPU data
  //   otherwise.
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.acceptable_buffer_backends = (phase == Phase::CUDA) ? "cuda" : "cpu";

  RCLCPP_INFO(
    get_logger(), "*** Starting the benchmark with the '%s' buffer backend ***",
    sub_opts.acceptable_buffer_backends.c_str());

  _sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "point_cloud", _defaultQoS,
    std::bind(&ZedCudaPcSubComponent::sub_callback, this, std::placeholders::_1),
    sub_opts);
}

bool ZedCudaPcSubComponent::process_on_gpu(
  const sensor_msgs::msg::PointCloud2 & cloud,
  float & x, float & y, float & z)
{
  const size_t size = static_cast<size_t>(cloud.row_step) * cloud.height;
  if (cloud.width == 0 || cloud.height == 0 || cloud.data.size() < size || !_stream) {
    return false;
  }

  // The middle point of the cloud. The cloud is organized like an image
  // (`height` rows of `width` points), or it is a single row of points when
  // it is unorganized (e.g. the voxelized point cloud of the ZED node)
  const size_t offset =
    (cloud.height / 2) * static_cast<size_t>(cloud.row_step) +
    (cloud.width / 2) * static_cast<size_t>(cloud.point_step);
  std::vector<uint8_t> point(cloud.point_step);

  // Placeholder for the GPU processing of a real application (e.g. a CUDA
  // kernel launched on `_stream`): copy the middle point back to CPU memory
  auto process = [&](const uint8_t * dev_ptr) {
      cudaError_t err = cudaMemcpyAsync(
        point.data(), dev_ptr + offset, cloud.point_step, cudaMemcpyDeviceToHost, _stream);
      if (err == cudaSuccess) {
        err = cudaStreamSynchronize(_stream);
      }
      if (err != cudaSuccess) {
        RCLCPP_ERROR(get_logger(), "CUDA error: %s", cudaGetErrorString(err));
        return false;
      }
      std::memcpy(&x, point.data() + _offsetX, sizeof(float));
      std::memcpy(&y, point.data() + _offsetY, sizeof(float));
      std::memcpy(&z, point.data() + _offsetZ, sizeof(float));
      return true;
    };

  // Always check the backend of the received message: a subscription that
  // accepts the CUDA backend receives CPU data when the zero-copy transport
  // cannot be used (e.g. publisher on another host)
  if (cloud.data.get_backend_type() == "cuda") {
    try {
      // Zero-copy: the read handle waits for the GPU work of the publisher on
      // `_stream` and gives read-only access to the GPU memory of the message.
      // Use the pointer only while the handle is alive.
      cuda_buffer_backend::ReadHandle rh =
        cuda_buffer_backend::from_input_buffer(cloud.data, _stream);
      return process(rh.get_ptr());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "CUDA buffer error: %s", e.what());
      return false;
    }
  }

  // CPU data: the buffer behaves like a std::vector<uint8_t> and must be
  // uploaded to GPU memory before being processed
  if (_devBufferSize < size) {
    if (_devBuffer) {
      cudaFree(_devBuffer);
    }
    _devBufferSize = 0;
    if (cudaMalloc(reinterpret_cast<void **>(&_devBuffer), size) != cudaSuccess) {
      _devBuffer = nullptr;
      RCLCPP_ERROR(get_logger(), "Cannot allocate %zu bytes of GPU memory", size);
      return false;
    }
    _devBufferSize = size;
  }
  if (cudaMemcpyAsync(
      _devBuffer, cloud.data.data(), size, cudaMemcpyHostToDevice,
      _stream) != cudaSuccess)
  {
    RCLCPP_ERROR(get_logger(), "Cannot upload the point cloud to GPU memory");
    return false;
  }
  return process(_devBuffer);
}

void ZedCudaPcSubComponent::sub_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  auto now = this->now();

  // Retrieve the position of the X, Y, and Z coordinates in each point from
  // the description of the fields of the first message
  if (_offsetX < 0) {
    for (const auto & field : cloud->fields) {
      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
        continue;
      }
      if (field.name == "x") {_offsetX = field.offset;}
      if (field.name == "y") {_offsetY = field.offset;}
      if (field.name == "z") {_offsetZ = field.offset;}
    }
    if (_offsetX < 0 || _offsetY < 0 || _offsetZ < 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "The point cloud does not contain the FLOAT32 fields 'x', 'y', and 'z'. Exiting.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Receiving %ux%u point clouds - point size: %u bytes", cloud->width,
      cloud->height, cloud->point_step);
  }

  const std::string backend = cloud->data.get_backend_type();
  TransportBenchmark & results = (_phase == Phase::CUDA) ? _cudaResults : _cpuResults;

  if (_phase == Phase::CUDA && backend != "cuda") {
    _cpuFallbackCount++;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Receiving CPU data while accepting the CUDA backend: the zero-copy transport is "
      "not available. Check that the 'cuda_buffer_backend' package is installed, that the "
      "RMW is 'rmw_fastrtps_cpp', and that the ZED node runs on the same host and GPU.");
  }

  // Access the point cloud on the GPU, as an accelerated node would do, and
  // read the coordinates of the middle point
  float x = std::numeric_limits<float>::quiet_NaN();
  float y = x;
  float z = x;
  bool ok = process_on_gpu(*cloud, x, y, z);
  // Time when the point cloud is available for the GPU processing
  auto ready = this->now();

  // ----> Period statistics update
  // The average frequency is computed from the average period: averaging the
  // instantaneous frequencies would overestimate it when messages arrive in
  // bursts.
  double fps = 0.0;
  if (_lastMsgTime.nanoseconds() != 0) {
    double period = (now - _lastMsgTime).seconds();
    fps = 1.0 / period;
    update_benchmark_stats(results.sub_period, period);
  }
  _lastMsgTime = now;
  // <---- Period statistics update

  // ----> Latency statistics update
  // From the publishing of the message to the availability of the point
  // cloud on the GPU of the subscriber
  rclcpp::Time cloud_ts(cloud->header.stamp, now.get_clock_type());
  double latency = (ready - cloud_ts).seconds();
  update_benchmark_stats(results.latency, latency);
  // <---- Latency statistics update

  // ----> CPU and GPU load statistics update
  double cpu_load = _cpuLoadAvg.load();
  double gpu_load = _gpuLoadAvg.load();
  update_benchmark_stats(results.cpu_load, cpu_load);
  update_benchmark_stats(results.gpu_load, gpu_load);
  // <---- CPU and GPU load statistics update

  _sampleCount++;
  RCLCPP_INFO(
    this->get_logger(),
    " *** [%s] Sample %d/%d - backend: %s - FPS: %.1f - Latency: %.4f sec - CPU: %.1f%% - "
    "GPU: %.1f%%",
    _phase == Phase::CUDA ? "CUDA" : "CPU", _sampleCount, _totSamples, backend.c_str(), fps,
    latency, cpu_load, gpu_load);

  // Print the coordinates of the middle point. They are NaN when the depth is
  // not available for that point (e.g. too close or too far)
  if (!ok) {
    RCLCPP_WARN(this->get_logger(), "     Middle point: not available");
  } else if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
    RCLCPP_INFO(
      this->get_logger(), "     Middle point: X: %.3f m - Y: %.3f m - Z: %.3f m", x, y, z);
  } else {
    RCLCPP_INFO(this->get_logger(), "     Middle point: invalid (no depth)");
  }

  // Check if we acquired the desired number of samples
  if (_sampleCount < _totSamples) {
    return;
  }

  if (_phase == Phase::CPU) {
    RCLCPP_INFO(this->get_logger(), "Received %d CPU samples. Unsubscribing...", _totSamples);
    _sub.reset();
    RCLCPP_INFO(this->get_logger(), "-----------------------------------");

    // Continue with the CUDA zero-copy subscriber
    create_subscriber(Phase::CUDA);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Received %d CUDA samples. Unsubscribing...", _totSamples);
  _sub.reset();

  // Calculate the final statistics
  print_benchmark_results();

  // Perform a clean shutdown of the node
  RCLCPP_INFO(this->get_logger(), "Shutting down the node...");
  _cpuGpuLoadThreadRunning = false;
  if (_cpuGpuLoadThread.joinable()) {
    _cpuGpuLoadThread.join();
  }
  rclcpp::shutdown();
}

void ZedCudaPcSubComponent::print_benchmark_results()
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
      RCLCPP_INFO(
        this->get_logger(),
        "+++ Logging benchmark results in CSV file: %s +++",
        _csvLogFile.c_str());

      csv_file << "BENCHMARK, SAMPLES, UNITS, AVG, MIN, MAX, STDDEV\n";
      csv_file.close();
    }
  }

  RCLCPP_INFO(this->get_logger(), "-----------------------------------");
  // Lambda to print the average frequency computed from the average period
  auto print_freq = [this](const BenchmarkTest & period) {
      if (period.avg_val > 0.0) {
        RCLCPP_INFO(this->get_logger(), "   * Avg frequency: %.2f Hz", 1.0 / period.avg_val);
      }
    };

  RCLCPP_INFO(this->get_logger(), "CPU Subscriber benchmark results:");
  calculate_benchmark_results(_cpuResults.sub_period);
  print_freq(_cpuResults.sub_period);
  calculate_benchmark_results(_cpuResults.latency);
  calculate_benchmark_results(_cpuResults.cpu_load);
  calculate_benchmark_results(_cpuResults.gpu_load);
  RCLCPP_INFO(this->get_logger(), "-----------------------------------");
  RCLCPP_INFO(this->get_logger(), "CUDA Subscriber benchmark results:");
  calculate_benchmark_results(_cudaResults.sub_period);
  print_freq(_cudaResults.sub_period);
  calculate_benchmark_results(_cudaResults.latency);
  calculate_benchmark_results(_cudaResults.cpu_load);
  calculate_benchmark_results(_cudaResults.gpu_load);
  RCLCPP_INFO(
    this->get_logger(), " - CPU data received while accepting the CUDA backend: %d/%d",
    _cpuFallbackCount, _totSamples);
  RCLCPP_INFO(this->get_logger(), "-----------------------------------");
  RCLCPP_INFO(this->get_logger(), "Benchmark comparison:");
  compare_benchmark_results(_cpuResults.latency, _cudaResults.latency);
  compare_benchmark_results(_cpuResults.sub_period, _cudaResults.sub_period);
  compare_benchmark_results(_cpuResults.cpu_load, _cudaResults.cpu_load);
  compare_benchmark_results(_cpuResults.gpu_load, _cudaResults.gpu_load);
  RCLCPP_INFO(this->get_logger(), "-----------------------------------");
}

double ZedCudaPcSubComponent::get_cpu_load()
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

  if (diffTotal == 0) {return 0.0;}
  return 100.0 * (1.0 - diffIdle / diffTotal);
}

double ZedCudaPcSubComponent::get_gpu_load()
{
  // Jetson Orin: the GPU load is exposed in sysfs, as percentage * 10
  std::ifstream file("/sys/devices/platform/gpu.0/load");
  if (file.is_open()) {
    int value = 0;
    file >> value;
    return value / 10.0;
  }

  // Jetson Thor and x86_64: use NVML, loaded at runtime
  static void * nvml_device = nullptr;
  static NvmlGetUtilizationFn get_utilization = nullptr;
  static bool nvml_init_done = false;
  if (!nvml_init_done) {
    nvml_init_done = true;
    void * lib = dlopen("libnvidia-ml.so.1", RTLD_LAZY);
    if (lib) {
      auto init = reinterpret_cast<NvmlInitFn>(dlsym(lib, "nvmlInit_v2"));
      auto get_handle =
        reinterpret_cast<NvmlGetHandleFn>(dlsym(lib, "nvmlDeviceGetHandleByIndex_v2"));
      get_utilization =
        reinterpret_cast<NvmlGetUtilizationFn>(dlsym(lib, "nvmlDeviceGetUtilizationRates"));
      if (!init || !get_handle || !get_utilization || init() != 0 ||
        get_handle(0, &nvml_device) != 0)
      {
        nvml_device = nullptr;
      }
    }
    if (!nvml_device) {
      RCLCPP_WARN(get_logger(), "GPU load not available on this platform");
    }
  }

  NvmlUtilization utilization{0, 0};
  if (nvml_device && get_utilization(nvml_device, &utilization) == 0) {
    return static_cast<double>(utilization.gpu);
  }
  return 0.0;
}

void ZedCudaPcSubComponent::cpu_gpu_load_callback()
{
  std::deque<double> cpu_queue, gpu_queue;

  while (_cpuGpuLoadThreadRunning && rclcpp::ok()) {
    cpu_queue.push_back(get_cpu_load());
    gpu_queue.push_back(get_gpu_load());

    // ----> Simple moving average
    while (cpu_queue.size() > static_cast<size_t>(_cpuGpuLoadAvgWndSize)) {cpu_queue.pop_front();}
    while (gpu_queue.size() > static_cast<size_t>(_cpuGpuLoadAvgWndSize)) {gpu_queue.pop_front();}

    _cpuLoadAvg = std::accumulate(cpu_queue.begin(), cpu_queue.end(), 0.0) / cpu_queue.size();
    _gpuLoadAvg = std::accumulate(gpu_queue.begin(), gpu_queue.end(), 0.0) / gpu_queue.size();
    // <---- Simple moving average

    std::this_thread::sleep_for(std::chrono::milliseconds(_cpuGpuLoadPeriod));
  }
}

}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedCudaPcSubComponent)
