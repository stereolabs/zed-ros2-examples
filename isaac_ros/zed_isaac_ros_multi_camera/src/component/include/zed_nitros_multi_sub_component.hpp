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

#ifndef ZED_NITROS_MULTI_SUB_COMPONENT_HPP_
#define ZED_NITROS_MULTI_SUB_COMPONENT_HPP_

#include <rcutils/logging_macros.h>

#include <atomic>
#include <thread>

#include <isaac_ros_managed_nitros/managed_nitros_subscriber.hpp>
#include <isaac_ros_nitros_image_type/nitros_image_view.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "nitros_multi_sub_visibility_control.hpp"

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

struct BenchmarkResults
{
  BenchmarkTest latency_dds;
  BenchmarkTest latency_nitros;
  BenchmarkTest sub_freq_dds;
  BenchmarkTest sub_freq_nitros;
  BenchmarkTest cpu_load_dds;
  BenchmarkTest cpu_load_nitros;
  BenchmarkTest gpu_load_dds;
  BenchmarkTest gpu_load_nitros;
};

class ZedNitrosMultiSubComponent : public rclcpp::Node
{
public:
  ZED_NITROS_MULTI_SUB_COMPONENT_PUBLIC
  explicit ZedNitrosMultiSubComponent(const rclcpp::NodeOptions & options);

  virtual ~ZedNitrosMultiSubComponent() {}

protected:
  // Read the parameters from the parameter server
  void read_parameters();

  // Create and enable the standard ROS 2 subscriber
  void create_dds_subscribers();

  // Create and enable the Nitros subscriber
  void create_nitros_subscriber();

  // Standard ROS 2 subscriber callback for the image topic
  void dds_sub_callback(const sensor_msgs::msg::Image::ConstSharedPtr & img, int source_index);

  // Nitros subscriber callback for the image topic
  void nitros_sub_callback(
      const nvidia::isaac_ros::nitros::NitrosImageView& img, int source_index);

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
  int _totSamples = 100;  // Total number of samples to process before exiting
  bool _debugNitros = false;  // Enable Nitros debug information
  int _cpuGpuLoadPeriod = 100;  // Period in milliseconds to retrieve CPU and GPU load statistics
  int _cpuGpuLoadAvgWndSize = 10;  // Size of the averaging window for CPU and GPU load statistics
  std::string _csvLogFile = "";  // If not empty, log the benchmark results in a CSV file with the specified name
  bool _disableDDSTest = false;  // Disable the DDS benchmark test
  std::vector<std::string> _camNames;  // Names of the cameras to subscribe to

  // Cameras
  int _numCams = 1;  // Number of cameras

  // Message type
  bool _isDepth = false;  // True if the topic is a depth map
  std::string _encoding;  // Encoding of the subscribed image topic

  // Standard subscriber to the image topic
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> _subArray;

  // Nitros subscriber to the image topic
  std::vector<std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
      nvidia::isaac_ros::nitros::NitrosImageView>>>
  _nitrosSubArray;

  // Time of the last received message
  rclcpp::Time _lastMsgTime;

  // CPU and GPU load monitoring
  //rclcpp::TimerBase::SharedPtr _cpuGpuLoadTimer;
  std::thread _cpuGpuLoadThread;
  std::atomic<bool> _cpuGpuLoadThreadRunning{true};
  std::atomic<double> _cpuLoadAvg{0.0};
  std::atomic<double> _gpuLoadAvg{0.0};

  // Benchmark results
  std::vector<BenchmarkResults> _benchmarkResults;
};

}  // namespace stereolabs

#endif  // ZED_NITROS_MULTI_SUB_COMPONENT_HPP_
