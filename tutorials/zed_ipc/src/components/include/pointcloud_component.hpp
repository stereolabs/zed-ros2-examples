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

#ifndef POINTCLOUD_COMPONENT_HPP_
#define POINTCLOUD_COMPONENT_HPP_

#include <string>
#include <vector>
#include <rcutils/logging_macros.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ipc_visibility_control.hpp"
#include "winavg.hpp"

namespace stereolabs
{

class PointCloudComponent : public rclcpp::Node
{
public:
  IPC_COMPONENT_PUBLIC
  explicit PointCloudComponent(const rclcpp::NodeOptions & options);

  virtual ~PointCloudComponent() {}

protected:
  void callback_pointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    const std::string & topic_name);

  void readParameters();
  void createSubscribers();

private:
  // ----> QoS
  // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
  rclcpp::QoS _qos;
  rclcpp::SubscriptionOptions _subOpt;
  // <---- QoS

  // ----> Parameters
  int _camCount;
  // <---- Parameters

  // ----> Topics
  std::string _pcTopicPrefix = "pointcloud_";
  // <---- Topics

  // ----> Subscribers
  std::vector<std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>>> _pcSubs;
  // <---- Subscribers

  // ----> Statistics
  std::vector<std::unique_ptr<WinAvg>> _stats;
  std::vector<std::chrono::high_resolution_clock::time_point> _times;
  std::vector<int> _counters;
  std::vector<bool> _firsts;
  // <---- Statistics
};

}  // namespace stereolabs

#endif  // POINTCLOUD_COMPONENT_HPP_
