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

#ifndef PERFORMANCE_TEST_COMPONENT_HPP_
#define PERFORMANCE_TEST_COMPONENT_HPP_

#include <string>
#include <vector>
#include <rcutils/logging_macros.h>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include <HardwareLoad.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <zed_msgs/msg/objects_stamped.hpp>

#include "ipc_visibility_control.hpp"
#include "winavg.hpp"

#include <chrono>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iterator>

using json = nlohmann::json;

namespace stereolabs
{

/* ########################################### CPU Struct ####################################*/

struct CPUStats {
  long user, nice, system, idle, iowait, irq, softirq, steal;
  long total_time() const { return user + nice + system + idle + iowait + irq + softirq + steal; }
  long active_time() const { return total_time() - (idle + iowait); }
};

// Read CPU stats from /proc/stat
CPUStats read_cpu_stats() {
  std::ifstream file("/proc/stat");
  std::string line;
  std::getline(file, line);
  file.close();

  std::istringstream iss(line);
  std::string cpu;
  CPUStats stats;

  if (!(iss >> cpu >> stats.user >> stats.nice >> stats.system >> stats.idle >> stats.iowait >> stats.irq >> stats.softirq >> stats.steal)) {
      return {}; // Return zero stats if read fails
  }
  return stats;
}

inline float computeAvg(const std::vector<float>& values) {
  if (values.empty()) {
      return 0.0f; // or throw an exception depending on your use case
  }
  
  float sum = std::accumulate(values.begin(), values.end(), 0.0f);
  return sum / static_cast<float>(values.size());
}

/* ############################################################################################*/

/* PerformanceTest node subscribes to either pointcloud or image messages (or both together), and returns performances statistics as a QA test for the ROS wrapper. */
class PerformanceTest : public rclcpp::Node
{
public:
  IPC_COMPONENT_PUBLIC
  explicit PerformanceTest(const rclcpp::NodeOptions & options);

  ~PerformanceTest();

protected:
 
  /* Pointcloud subscribing  callback function. */
  void callback_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string& topic_name);

  /* Image subscribing  callback function. */
  void callback_image(const sensor_msgs::msg::Image::SharedPtr msg, const std::string& topic_name);

  /* ZED Object subscribing  callback function. */
  void callback_obj(const zed_msgs::msg::ObjectsStamped msg, const std::string& topic_name);

  /* Function that updates the parameters to the test desired configuration. */
  void readParameters();

  /* Function that parses the yaml file into a jason file. */
  void parseYamlNode(const YAML::Node &yaml_node, json &json_node);

  /* Function that updates the parameters to the test desired configuration. */
  void createSubscribers();

  /* Function that saves test results data to a json file. */
  void saveResults();


private:
  // ----> QoS
  // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
  rclcpp::QoS _qos;
  rclcpp::SubscriptionOptions _subOpt;
  // <---- QoS

  // ----> Parameters

   /* Start time. */
  rclcpp::Time _startTime;

  /* Number of cameras used for the test. */
  int _camCount;

  /* Whether Hardware Load started. */
  bool _has_hardware_load_started = false;

  /* File path to store the results data. */
  std::string _resultsFilePath;

  /* Config File path. */
  std::string _configFilePath;

  /* Current test name. */
  std::string _testName;

  /* Whether to subscribe to pointcloud messages for the test. */
  bool _usePointcloud = true;

  /* Whether to subscribe to image messages for the test. */
  bool _useImage = false;

  /* Whether to subscribe to object messages for the test */
  bool _useObj = false;

  /* Whether to launch to RVIZ for the test. */
  bool _useRVIZ = false;

  // <---- Parameters

 
  // ----> Topics

  /* Pointcloud topic prefix. */
  std::string _pcTopicPrefix = "pointcloud_";

  /* Image topic prefix. */
  std::string _imageTopicPrefix = "image_";

  /* Object topic prefix. */
  std::string _objTopicPrefix = "objects_";

  // <---- Topics

  // ----> Timers 

  /* CPU timer. */
  rclcpp::TimerBase::SharedPtr _CPUTimer;

  // <---- Timers

  // ----> Subscribers

  /* Pointcloud subscriber. */
  std::vector<std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>>> _pcSubs;

  /* Image subscriber. */
  std::vector<std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>>> _imageSubs;

  /* Object subscriber */
  std::vector<std::shared_ptr<rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>>> _objSubs;

  // <---- Subscribers

  std::shared_ptr<rclcpp::SyncParametersClient> param_client_;

  // ----> Statistics

  /* Average Frequencies for pointcloud and image. */
  std::vector<std::unique_ptr<WinAvg>> _pointcloudFrequencies, _imageFrequencies, 
    _objFrequencies;

  /* Average CPU load. */
  std::unique_ptr<WinAvg> _cpu, _gpu;
  HardwareLoad _hardwareLoad;

  /* Current Bandwidths for pointcloud and image. */
  std::vector<double> _pointcloudBandwidthAvg, _imageBandwidthAvg;

  /* Current times for pointcloud and image. */
  std::vector<std::chrono::steady_clock::time_point> _times, _imageTimes, _objTimes;

  /* Current images counters for pointcloud and image. */
  std::vector<int> _counters ,_imageCounters, _objCounters;

  /* Whether a first message was received for the pointcloud and image messages. */
  std::vector<bool> _firsts, _imageFirsts, _objFirsts;

  // <---- Statistics
};

}  // namespace stereolabs

#endif  // PERFORMANCE_TEST_COMPONENT_HPP_
