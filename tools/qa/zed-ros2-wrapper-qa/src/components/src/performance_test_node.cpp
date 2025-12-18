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

#include "performance_test_node.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;
using json = nlohmann::json;

namespace stereolabs
{

 /* ##################################### Performance Node ######################################*/

PerformanceTest::PerformanceTest(const rclcpp::NodeOptions & options)
: Node("performance_test_node", options)
, _qos(1)
{
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), "   Performance test Node Component ");
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

  _pointcloudFrequencies.resize(_camCount);
  _imageFrequencies.resize(_camCount);
  _objFrequencies.resize(_camCount);
  _cpu = std::make_unique<WinAvg>(100);
  _gpu = std::make_unique<WinAvg>(100);

  for(int i = 0; i < _camCount; i++) {
    _pointcloudFrequencies[i] = std::make_unique<WinAvg>(500);
    _imageFrequencies[i] = std::make_unique<WinAvg>(500);
    _objFrequencies[i] = std::make_unique<WinAvg>(500);
  }

  _pointcloudBandwidthAvg.resize(_camCount);
  _imageBandwidthAvg.resize(_camCount);
    
  _times.resize(_camCount);
  _firsts.resize(_camCount,false);
  _counters.resize(_camCount,0);

  _imageTimes.resize(_camCount);
  _imageFirsts.resize(_camCount,false);
  _imageCounters.resize(_camCount,0);

  _objTimes.resize(_camCount);
  _objFirsts.resize(_camCount,false);
  _objCounters.resize(_camCount,0);

  createSubscribers();

  //Set up Performance test running time timer
  _startTime = now();

  // Ensure destructor is called on shutdown
  rclcpp::on_shutdown([this]() {
    this->saveResults();
  });

}

PerformanceTest::~PerformanceTest() {
  saveResults();
}

void PerformanceTest::parseYamlNode(const YAML::Node &yaml_node, json &json_node) {
  if (yaml_node.IsMap()) {
    for (auto it = yaml_node.begin(); it != yaml_node.end(); ++it) {
        json_node[it->first.as<std::string>()] = json();
        parseYamlNode(it->second, json_node[it->first.as<std::string>()]);
    }
} else if (yaml_node.IsSequence()) {
    for (const auto &item : yaml_node) {
        json_node.push_back(json());
        parseYamlNode(item, json_node.back());
    }
} else {
    json_node = yaml_node.as<std::string>();
}
}

void PerformanceTest::saveResults() {

    //Write results to JSON file

    std::string _filename = _resultsFilePath +".json";
    std::ifstream input_file(_filename);
    json data;

    YAML::Node yaml_data = YAML::LoadFile(_configFilePath);
    json json_data;
    parseYamlNode(yaml_data, json_data);
    std::cout<<json_data<<std::endl;

    _hardwareLoad.join();

    if (input_file) {
      try {
          if (input_file.peek() == std::ifstream::traits_type::eof()) {
              data = json::array();
          } else {
              input_file >> data;
          }
      } catch (...) {
          RCLCPP_WARN(this->get_logger(), "Error reading JSON file, starting fresh.");
          data = json::array();
      }
    } else {
      data = json::array();
    }
    input_file.close();

    auto cpu_measures = _hardwareLoad.getCPUloadMeasures();
    auto gpu_measures = _hardwareLoad.getGPUloadMeasures();

    json results;
    results["test_name"] = _testName;
    results["publish_pointcloud"] = _usePointcloud;
    results["publish_image"] = _useImage;
    results["publish_obj"] = _useObj;
    results["use_rviz"] = _useRVIZ;
    results["cpu_load"] = computeAvg(cpu_measures);
    results["gpu_load"] = computeAvg(gpu_measures);
    results["grab_rate"] = json_data["/**"]["ros__parameters"]["general"]["grab_frame_rate"];
    results["camera_count"] = _camCount;
    results["depth_mode"] = json_data["/**"]["ros__parameters"]["depth"]["depth_mode"];
    results["pointcloud_resolution"] = json_data["/**"]["ros__parameters"]["depth"]["point_cloud_res"];
    results["pointcloud_frequency"] = json_data["/**"]["ros__parameters"]["depth"]["point_cloud_freq"];
    results["camera_model"] = json_data["/**"]["ros__parameters"]["general"]["camera_model"];
    results["grab_resolution"] = json_data["/**"]["ros__parameters"]["general"]["grab_resolution"];
    results["pub_downscale_factor"] = json_data["/**"]["ros__parameters"]["general"]["pub_downscale_factor"];
    results["pub_resolution"] = json_data["/**"]["ros__parameters"]["general"]["pub_resolution"];

    if (json_data["/**"]["ros__parameters"]["pos_tracking"]["pos_tracking_enabled"] == "true")
    {
      results["positional_tracking_mode"] = json_data["/**"]["ros__parameters"]["pos_tracking"]["pos_tracking_mode"];
    }

     if (json_data["/**"]["ros__parameters"]["object_detection"]["od_enabled"] == "true")
    {
      results["object_detection_model"] = json_data["/**"]["ros__parameters"]["object_detection"]["detection_model"];
    }


    if (_usePointcloud){
      for (int i =0; i < _camCount; i++) {
        results["camera"+std::to_string(i)+"_pointcloud_frequency"] = _pointcloudFrequencies[i]->getAvg();
        results["camera"+std::to_string(i)+"_pointcloud_bandwidth"] = _pointcloudBandwidthAvg[i];
        results["camera"+std::to_string(i)+"_pointcloud_std"] = _pointcloudFrequencies[i]->get_std_dev();
      }
    }

    if (_useImage){
      for (int i =0; i < _camCount; i++) {
        results["camera"+std::to_string(i)+"_image_frequency"] = _imageFrequencies[i]->getAvg();
        results["camera"+std::to_string(i)+"_image_bandwidth"] = _imageBandwidthAvg[i];
        results["camera"+std::to_string(i)+"_image_std"] = _imageFrequencies[i]->get_std_dev();
      }
    }

    if (_useObj) {
      for (int i =0; i < _camCount; i++) {
        results["camera"+std::to_string(i)+"_object_frequency"] = _objFrequencies[i]->getAvg();
        results["camera"+std::to_string(i)+"_object_std"] = _objFrequencies[i]->get_std_dev();
      }
    }


    data.push_back(results);

    RCLCPP_INFO_STREAM(get_logger(), "Saving results to file path: "<<_filename.c_str());

    std::ofstream output_file(_filename);
    output_file << data.dump(4); 
}

void PerformanceTest::readParameters()
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

  // ----> Get the file path for printing the results
  param_name = "results_file_path";
  declare_parameter(param_name, "");
  if (!get_parameter(param_name, _resultsFilePath)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << param_name
        << "' is not available or is not valid, using the default value: 1");
  } else {
    RCLCPP_INFO_STREAM(get_logger(), " * Writing results to " << _resultsFilePath);
  }
  // <---- Get the file path for printing the results

   // ----> Get the ROS yaml config file path for printing the current test configuration
   param_name = "ros_params_override_path";
   declare_parameter(param_name, "");
   if (!get_parameter(param_name, _configFilePath)) {
     RCLCPP_WARN_STREAM(
       get_logger(),
       "The parameter '"
         << param_name
         << "' is not available or is not valid, using the default value: 1");
   } else {
     RCLCPP_INFO_STREAM(get_logger(), " * Retrieving test configuration from " << _configFilePath);
   }
   // <---- Get the ROS yaml config file path for printing the current test configuration

  // ----> Get the test name 
   param_name = "test_name";
   declare_parameter(param_name, "");
   if (!get_parameter(param_name, _testName)) {
     RCLCPP_WARN_STREAM(
       get_logger(),
       "The parameter '"
         << param_name
         << "' is not available or is not valid, using the default value: 1");
   } else {
     RCLCPP_INFO_STREAM(get_logger(), " * Test name: " << _testName);
   }
   // <---- Get the test name

   // ----> Get the use_image parameter
   param_name = "use_image";
   declare_parameter(param_name, false);
   if (!get_parameter(param_name, _useImage)) {
     RCLCPP_WARN_STREAM(
       get_logger(),
       "The parameter '"
         << param_name
         << "' is not available or is not valid, using the default value: 1");
   } else {
     RCLCPP_INFO_STREAM(get_logger(), " * Use Image: " << _useImage);
   }
   // <---- Get the use_image parameter

    // ----> Get the use_pointcloud parameter
    param_name = "use_pointcloud";
    declare_parameter(param_name, false);
    if (!get_parameter(param_name, _usePointcloud)) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "The parameter '"
          << param_name
          << "' is not available or is not valid, using the default value: 1");
    } else {
      RCLCPP_INFO_STREAM(get_logger(), " * Use Pointcloud: " << _usePointcloud);
    }
    // <---- Get the use_pointcloud parameter

    // ----> Get the use_obj parameter
    param_name = "use_obj";
    declare_parameter(param_name, false);
    if (!get_parameter(param_name, _useObj)) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "The parameter '"
          << param_name
          << "' is not available or is not valid, using the default value: 0");
    } else {
      RCLCPP_INFO_STREAM(get_logger(), " * Use OBJ: " << _useObj);
    }
    // <---- Get the use_obj parameter

   // ----> Get the use_rviz parameter
   param_name = "use_rviz";
   declare_parameter(param_name, false);
   if (!get_parameter(param_name, _useRVIZ)) {
     RCLCPP_WARN_STREAM(
       get_logger(),
       "The parameter '"
         << param_name
         << "' is not available or is not valid, using the default value: 1");
   } else {
     RCLCPP_INFO_STREAM(get_logger(), " * Use Rviz: " << _useRVIZ);
   }
   // <---- Get the use_rviz parameter
}

void PerformanceTest::createSubscribers()
{
  std::string topic_name;
  _qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  if (_usePointcloud)
  {
    // ----> Create the pointcloud subscribers
    for (int i = 0; i < _camCount; i++) {
      topic_name = _pcTopicPrefix + std::to_string(i);
      RCLCPP_INFO_STREAM(get_logger(), " * Subscribing to topic: " << topic_name);

      std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr msg)> bound_callback_func =
      std::bind(&PerformanceTest::callback_pointcloud, this, _1, topic_name);

      auto sub = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_name, _qos, bound_callback_func, _subOpt);
      _pcSubs.push_back(sub);
    }
    // <---- Create the pointcloud subscribers
  }
  

  if (_useImage)
  {
   // ----> Create the image subscribers
   for (int i = 0; i < _camCount; i++) {
    topic_name = _imageTopicPrefix + std::to_string(i);
    RCLCPP_INFO_STREAM(get_logger(), " * Subscribing to topic: " << topic_name);

    std::function<void(const sensor_msgs::msg::Image::SharedPtr msg)> image_bound_callback_func =
    std::bind(&PerformanceTest::callback_image, this, _1, topic_name);

    auto sub = create_subscription<sensor_msgs::msg::Image>(
      topic_name, _qos, image_bound_callback_func, _subOpt);
    _imageSubs.push_back(sub);
   }
   // <---- Create the image subscribers
  }

  if (_useObj)
  {
    // ----> Create the OBJ subscribers
    for (int i = 0; i < _camCount; i++) {
      topic_name = _objTopicPrefix + std::to_string(i);
      RCLCPP_INFO_STREAM(get_logger(), " * Subscribing to topic: " << topic_name);

      std::function<void(const zed_msgs::msg::ObjectsStamped)> bound_callback_func =
      std::bind(&PerformanceTest::callback_obj, this, _1, topic_name);

      auto sub = create_subscription<zed_msgs::msg::ObjectsStamped>(
      topic_name, _qos, bound_callback_func, _subOpt);
      _objSubs.push_back(sub);
    }
    // <---- Create the OBJ subscribers
  }
}

void PerformanceTest::callback_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string& topic_name)
{
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
  if (!_firsts[idx]) {
    _times[idx] = std::chrono::steady_clock::now();    // Set the start time point
    _firsts[idx] = true;
    return;
  }

  auto now = std::chrono::steady_clock::now();
  double elapsed= std::chrono::duration<double>(now - _times[idx]).count();
  _counters[idx]++;
  if (elapsed >= 2) {
    _times[idx] = now;
    double freq = _counters[idx] / elapsed;
    double avg_freq = _pointcloudFrequencies[idx]->addValue(freq);

    static double bw_scale = 8. / (1024. * 1024.);

    int data_size = msg->data.size();
    double bw = freq * bw_scale * data_size;
    _pointcloudBandwidthAvg[idx] = avg_freq * bw_scale * data_size;
  
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << " Point Cloud #"
             << _counters[idx] << " - Freq: " << freq << " Hz (Avg: " << avg_freq
            << " Hz) - BW: " << bw << " Mbps (Avg: " << _pointcloudBandwidthAvg[idx]
            << " Mbps) - Msg size: " << data_size / (1024. * 1024.) << " MB";

    rclcpp::Time msg_ts = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);
    rclcpp::Time ros_ts = rclcpp::Clock().now();

    auto latency_ns = (ros_ts - msg_ts).nanoseconds();

    ss << " - Point Cloud latency: " << latency_ns / 1e6 << " ms";

    RCLCPP_INFO_STREAM(get_logger(), ss.str());
    _counters[idx] = 0;
  }
  
  // <---- Calculate statistics
}

void PerformanceTest::callback_image(const sensor_msgs::msg::Image::SharedPtr msg, const std::string& topic_name)
{
  // ----> Extract index X from "image_X"
  int idx = -1;
  std::size_t pos = topic_name.find(_imageTopicPrefix);
  if (pos != std::string::npos) {
    std::string index_str = topic_name.substr(pos + _imageTopicPrefix.length());
    idx = std::stoi(index_str);
  } else {
    return;
  }
  // <---- Extract index X from "image_X"

  if (!_has_hardware_load_started) {
    RCLCPP_INFO_STREAM(get_logger(), "Starting hardware measures");
    _hardwareLoad.launch();
    _has_hardware_load_started = true;
  }

  // ----> Calculate statistics
  if (!_imageFirsts[idx]) {
    _imageTimes[idx] = std::chrono::steady_clock::now();    // Set the start time point
    _imageFirsts[idx] = true;

    return;
  }

  auto now = std::chrono::steady_clock::now();
  double elapsed= std::chrono::duration<double>(now - _imageTimes[idx]).count();
  _imageCounters[idx]++;
  if (elapsed >= 2) {
    _imageTimes[idx] = now;
    double freq = _imageCounters[idx] / elapsed;
    double avg_freq = _imageFrequencies[idx]->addValue(freq);

    static double bw_scale = 2 / (1024. * 1024.);

    int data_size = msg->step * msg->height;
    double bw = freq * bw_scale * data_size;
    _imageBandwidthAvg[idx] = avg_freq * bw_scale * data_size;
  
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << " Image #"
             << _imageCounters[idx] << " - Freq: " << freq << " Hz (Avg: " << avg_freq
            << " Hz) - BW: " << bw << " Mbps (Avg: " << _imageBandwidthAvg[idx]
            << " Mbps) - Msg size: " << data_size / (1024. * 1024.) << " MB";

    RCLCPP_INFO_STREAM(get_logger(), ss.str());
    _imageCounters[idx] = 0;
  }
  
  // <---- Calculate statistics
}

void PerformanceTest::callback_obj(const zed_msgs::msg::ObjectsStamped msg, const std::string& topic_name)
{
  // ----> Extract index X from "obj_X"
  int idx = -1;
  std::size_t pos = topic_name.find(_objTopicPrefix);
  if (pos != std::string::npos) {
    std::string index_str = topic_name.substr(pos + _objTopicPrefix.length());
    idx = std::stoi(index_str);
  } else {
    return;
  }
  // <---- Extract index X from "obj_X"

  // ----> Calculate statistics
  if (!_objFirsts[idx]) {
    _objTimes[idx] = std::chrono::steady_clock::now();    // Set the start time point
    _objFirsts[idx] = true;
    return;
  }

  auto now = std::chrono::steady_clock::now();
  double elapsed= std::chrono::duration<double>(now - _objTimes[idx]).count();

  _objCounters[idx]++;
  if (elapsed >= 2) {

    _objTimes[idx] = now;
    double freq = _objCounters[idx] / elapsed;
    double avg_freq = _objFrequencies[idx]->addValue(freq);

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << " Objects: #"
             << _objCounters[idx] << " - Freq: " << freq << " Hz (Avg: " << avg_freq
            << " Hz)" ;

    rclcpp::Time msg_ts = rclcpp::Time(msg.header.stamp.sec, msg.header.stamp.nanosec);
    rclcpp::Time ros_ts = rclcpp::Clock().now();

    auto latency_ns = (ros_ts - msg_ts).nanoseconds();

    ss << " - Object latency: " << latency_ns / 1e6 << " ms";

    RCLCPP_INFO_STREAM(get_logger(), ss.str());
    _objCounters[idx] = 0;
  }
  // <---- Calculate statistics
}

/* ############################################################################################*/


}  // namespace stereolabs



#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::PerformanceTest)
