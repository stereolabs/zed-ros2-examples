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

#ifndef ZED_CUDA_PC_SUB__COMPONENT_HPP_
#define ZED_CUDA_PC_SUB__COMPONENT_HPP_

#include <cuda_runtime_api.h>

#include <atomic>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "cuda_pc_sub_visibility_control.hpp"

namespace stereolabs
{

struct BenchmarkTest
{
  std::string name = "";
  std::string units = "";
  std::vector<double> values;

  double sum = 0.0;
  double min_val = std::numeric_limits<double>::max();
  double max_val = 0.0;
  double avg_val = 0.0;
  double std_dev_val = 0.0;
};

// The statistics collected for one transport
struct TransportBenchmark
{
  BenchmarkTest latency;
  BenchmarkTest sub_period;  // Time between two messages
  BenchmarkTest cpu_load;
  BenchmarkTest gpu_load;
};

// The two benchmark phases: the subscriber accepts only CPU data, then also
// the CUDA buffer backend
enum class Phase { CPU, CUDA };

class ZedCudaPcSubComponent : public rclcpp::Node
{
public:
  ZED_CUDA_PC_SUB_COMPONENT_PUBLIC
  explicit ZedCudaPcSubComponent(const rclcpp::NodeOptions & options);

  virtual ~ZedCudaPcSubComponent();

protected:
  // Read the parameters from the parameter server
  void read_parameters();

  // Create the subscriber for the given benchmark phase
  void create_subscriber(Phase phase);

  // Subscriber callback for the point cloud topic
  void sub_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud);

  // Access the point cloud on the GPU, as an accelerated node would do, and
  // read the coordinates of its middle point. CUDA-backed messages are read
  // directly, CPU messages are uploaded to GPU memory first.
  bool process_on_gpu(
    const sensor_msgs::msg::PointCloud2 & cloud,
    float & x, float & y, float & z);

  // Initialize the benchmark results statistics
  void initialize_benchmark_results();

  // Update benchmark results statistics
  void update_benchmark_stats(BenchmarkTest & benchmark, double new_value);

  // Calculate and print the benchmark results statistics
  void calculate_benchmark_results(BenchmarkTest & benchmark);

  // Compare two benchmark results and print the comparison
  void compare_benchmark_results(
    const BenchmarkTest & benchmark1,
    const BenchmarkTest & benchmark2);

  // Compose the final benchmark results and print them
  void print_benchmark_results();

  // Retrieve CPU load
  double get_cpu_load();

  // Retrieve GPU load
  double get_gpu_load();

  // Thread function to periodically retrieve CPU and GPU load
  void cpu_gpu_load_callback();

private:
  // QoS parameters
  rclcpp::QoS _defaultQoS;

  // Parameters
  int _totSamples = 100;  // Number of samples to process for each benchmark phase
  bool _debugCuda = false;  // Enable debug information
  int _cpuGpuLoadPeriod = 100;  // Period in milliseconds to retrieve CPU and GPU load statistics
  int _cpuGpuLoadAvgWndSize = 10;  // Size of the averaging window for CPU and GPU load statistics
  std::string _csvLogFile = "";  // If not empty, log the benchmark results in a CSV file with the specified name

  // Current benchmark phase
  Phase _phase = Phase::CPU;
  int _sampleCount = 0;  // Samples received in the current phase
  int _cpuFallbackCount = 0;  // CPU messages received while accepting the CUDA backend

  // Offsets of the X, Y and Z coordinates in a point of the cloud
  int _offsetX = -1;
  int _offsetY = -1;
  int _offsetZ = -1;

  // Subscriber to the point cloud topic
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub;

  // CUDA stream used to access the received point clouds on the GPU
  cudaStream_t _stream = nullptr;

  // GPU memory used to upload the point clouds received as CPU data
  uint8_t * _devBuffer = nullptr;
  size_t _devBufferSize = 0;

  // Time of the last received message
  rclcpp::Time _lastMsgTime;

  // CPU and GPU load monitoring
  std::thread _cpuGpuLoadThread;
  std::atomic<bool> _cpuGpuLoadThreadRunning{true};
  std::atomic<double> _cpuLoadAvg{0.0};
  std::atomic<double> _gpuLoadAvg{0.0};

  // Benchmark results
  TransportBenchmark _cpuResults;
  TransportBenchmark _cudaResults;
};

}  // namespace stereolabs

#endif  // ZED_CUDA_PC_SUB__COMPONENT_HPP_
