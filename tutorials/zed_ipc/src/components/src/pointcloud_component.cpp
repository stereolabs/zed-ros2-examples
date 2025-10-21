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

#include "pointcloud_component.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace stereolabs
{

PointCloudComponent::PointCloudComponent(const rclcpp::NodeOptions & options)
: Node("pointcloud_node", options)
    , _qos(1)
{
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), "   Point Cloud Sub Component ");
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "********************************");

  /* Note: it is very important to use a QOS profile for the subscriber that is
   * compatible with the QOS profile of the publisher. The ZED component node
   * uses a default QoS profile with reliability set as "RELIABLE" and
   * durability set as "VOLATILE". To be able to receive the subscribed topic
   * the subscriber must use compatible parameters.
   */

  // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings

  readParameters();

  _stats.resize(_camCount);
  for (int i = 0; i < _camCount; i++) {
    _stats[i] = std::make_unique<WinAvg>(500);
  }
  _times.resize(_camCount);
  _firsts.resize(_camCount, false);
  _counters.resize(_camCount, 0);

  createSubscribers();
}

void PointCloudComponent::readParameters()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;

  // ----> Get the number of cameras
  std::string param_name = "cam_count";
  descriptor.description = "Number of cameras";
  descriptor.additional_constraints = "Value >= 1";
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = 1;
  range.to_value = 255;
  descriptor.integer_range.push_back(range);
  descriptor.name = param_name;
  descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  declare_parameter(param_name, rclcpp::ParameterValue(1), descriptor);
  if (!get_parameter(param_name, _camCount)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << param_name
        << "' is not available or is not valid, using the default value: 1");
  } else {
    RCLCPP_INFO_STREAM(get_logger(), " * Subscribing to " << _camCount << " cameras");
  }
  // <---- Get the number of cameras
}

void PointCloudComponent::createSubscribers()
{
  // ----> Create the subscribers
  for (int i = 0; i < _camCount; i++) {
    std::string topic_name = _pcTopicPrefix + std::to_string(i);
    RCLCPP_INFO_STREAM(get_logger(), " * Subscribing to topic: " << topic_name);

    std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr msg)> bound_callback_func =
      std::bind(&PointCloudComponent::callback_pointcloud, this, _1, topic_name);

    _qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    auto sub = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_name, _qos, bound_callback_func, _subOpt);
    _pcSubs.push_back(sub);
  }
  // <---- Create the subscribers
}

void PointCloudComponent::callback_pointcloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  const std::string & topic_name)
{
  RCLCPP_INFO_STREAM(get_logger(), "Received point cloud from topic: " << topic_name);

  // ----> Extract index X from "pointcloud_X"
  int idx = -1;
  std::size_t pos = topic_name.find(_pcTopicPrefix);
  if (pos != std::string::npos) {
    std::string index_str = topic_name.substr(pos + _pcTopicPrefix.length());
    idx = std::stoi(index_str);
  } else {
    return;
  }
  // <---- Extract index X from "pointcloud_X"

  // ----> Calculate statistics
  if (_firsts[idx]) {
    _times[idx] =
      std::chrono::high_resolution_clock::now();    // Set the start time point
    _firsts[idx] = false;
    return;
  }

  auto now = std::chrono::high_resolution_clock::now();
  double elapsed_usec =
    std::chrono::duration_cast<std::chrono::microseconds>(now - _times[idx])
    .count();
  _times[idx] = now;

  double freq = 1e6 / elapsed_usec;
  double avg_freq = _stats[idx]->addValue(freq);

  if (freq > 60.0) {
    return;
  }

  static double bw_scale = 8. / (1024. * 1024.);

  int data_size = msg->data.size();
  double bw = freq * bw_scale * data_size;
  double bw_avg = avg_freq * bw_scale * data_size;

  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << " #"
     << ++_counters[idx] << " - Freq: " << freq << " Hz (Avg: " << avg_freq
     << " Hz) - BW: " << bw << " Mbps (Avg: " << bw_avg
     << " Mbps) - Msg size: " << data_size / (1024. * 1024.) << " MB";

  RCLCPP_INFO_STREAM(get_logger(), ss.str());
  // <---- Calculate statistics
}

}  // namespace stereolabs


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::PointCloudComponent)
