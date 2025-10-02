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

#ifndef ZED_NITROS_SUB__COMPONENT_HPP_
#define ZED_NITROS_SUB__COMPONENT_HPP_

#include <rcutils/logging_macros.h>

#include <isaac_ros_managed_nitros/managed_nitros_subscriber.hpp>
#include <isaac_ros_nitros_image_type/nitros_image_view.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "nitros_sub_visibility_control.hpp"

namespace stereolabs
{

struct BenchmarkTest
{
  std::string name;
  std::vector<double> values;

  double sum;
  double min_val;
  double max_val;
  double avg_val;
  double std_dev_val;
};

struct BenchmarkResults
{
  BenchmarkTest latency_dds;
  BenchmarkTest latency_nitros;
  BenchmarkTest sub_freq_dds;
  BenchmarkTest sub_freq_nitros;
};

class ZedNitrosSubComponent : public rclcpp::Node
{
public:
  ZED_NITROS_SUB_COMPONENT_PUBLIC
  explicit ZedNitrosSubComponent(const rclcpp::NodeOptions & options);

  virtual ~ZedNitrosSubComponent() {}

protected:
  // Read the parameters from the parameter server
  void read_parameters();

  // Create and enable the standard ROS 2 subscriber
  void create_dds_subscriber();

  // Create and enable the Nitros subscriber
  void create_nitros_subscriber();

// Standard ROS 2 subscriber callback for the image topic
  void dds_sub_callback(const sensor_msgs::msg::Image::ConstSharedPtr & img);

  // Nitros subscriber callback for the image topic
  void nitros_sub_callback(
    const nvidia::isaac_ros::nitros::NitrosImageView & img);

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

private:
  // QoS parameters
  rclcpp::QoS _defaultQoS;

  // Parameters
  int _totSamples = 100;  // Total number of samples to process before exiting
  bool _debugNitros = false;  // Enable Nitros debug information

  // Message type
  bool _isDepth = false;  // True if the topic is a depth map
  std::string _encoding;  // Encoding of the subscribed image topic

  // Standard subscriber to the image topic
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub;

  // Nitros subscriber to the image topic
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
      nvidia::isaac_ros::nitros::NitrosImageView>>
  _nitrosSub;

  // Time of the last received message
  rclcpp::Time _lastMsgTime;

  // Benchmark results
  BenchmarkResults _benchmarkResults;
};

}  // namespace stereolabs

#endif  // ZED_NITROS_SUB__COMPONENT_HPP_
