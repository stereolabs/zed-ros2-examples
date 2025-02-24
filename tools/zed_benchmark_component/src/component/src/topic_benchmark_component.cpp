// Copyright 2024 Stereolabs
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

#include "topic_benchmark_component.hpp"

#include <rcutils/logging_macros.h>

#include <chrono>
#include <iostream>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_overriding_options.hpp>
#include <rclcpp/time.hpp>
#include <sstream>

using namespace std::placeholders;

namespace stereolabs
{

const int QOS_QUEUE_SIZE = 1;

TopicBenchmarkComponent::TopicBenchmarkComponent(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("topic_benchmark", options)
{
  mTopicAvailable.store(false);

  init();

  auto pub_opt = rclcpp::PublisherOptions();
  pub_opt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();

  std::string pub_topic_name =
    /*std::string("~/") + */ mTopicName + std::string("_stats");
  mPub = create_publisher<
    zed_topic_benchmark_interfaces::msg::BenchmarkStatsStamped>(
    pub_topic_name, rclcpp::QoS(QOS_QUEUE_SIZE), pub_opt);
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised on topic: " << mPub->get_topic_name());
}

TopicBenchmarkComponent::~TopicBenchmarkComponent()
{
  if (mTopicTimer) {
    mTopicTimer->cancel();
  }
}

void TopicBenchmarkComponent::init()
{
  getParameters();

  RCLCPP_INFO(get_logger(), "*** START BENCHMARK ***");

  if (!mTopicAvailable.load()) {
    mTopicTimer = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::milliseconds(500)),
      std::bind(&TopicBenchmarkComponent::updateTopicInfo, this));
  }
}

template<typename T>
void TopicBenchmarkComponent::getParam(
  std::string paramName, T defValue,
  T & outVal, std::string log_info,
  bool dynamic)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = !dynamic;

  try {
    declare_parameter(paramName, rclcpp::ParameterValue(defValue), descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Exception: " << ex.what());
  }

  if (!get_parameter(paramName, outVal)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << defValue);
  }

  if (!log_info.empty()) {
    RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
  }
}

void TopicBenchmarkComponent::getParameters()
{
  RCLCPP_INFO(get_logger(), "***** Benchmark parameters *****");

  getParam("topic_name", DEFAULT_TOPIC_NAME, mTopicName, "* Topic name: ");
  if (mTopicName == DEFAULT_TOPIC_NAME) {
    RCLCPP_WARN(
      get_logger(),
      "Please remap the parameter 'topic_name' with the name of the "
      "parameter to benchmark.\n"
      "e.g. 'ros2 run zed_topic_benchmark zed_topic_benchmark --ros-args -p "
      "topic_name:=/zed2i/zed_node/rgb/image_rect_color'");
  }
  getParam("avg_win_size", mWinSize, mWinSize, "Average window size: ");
  mAvgFreq.setNewSize(mWinSize);
  getParam("use_ros_log", mUseRosLog, mUseRosLog, "ROS Log: ");
}

void TopicBenchmarkComponent::updateTopicInfo()
{
  mTopicAvailable.store(false);

  std::map<std::string, std::vector<std::string>> topic_infos =
    this->get_topic_names_and_types();
  for (const auto & topic_it : topic_infos) {
    std::string topic_name = topic_it.first;

    std::vector<std::string> topicTypes = topic_it.second;

    if (topic_name == mTopicName) {
      // iterate over all topic types
      for (const auto & topic_type : topicTypes) {
        mTopicAvailable.store(true);
        RCLCPP_INFO_STREAM(
          get_logger(), "Found topic: '" << mTopicName
                                         << "' of type: '"
                                         << topic_type << "'");

        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.qos_overriding_options =
          rclcpp::QosOverridingOptions::with_default_policies();

        std::shared_ptr<rclcpp::GenericSubscription> sub =
          create_generic_subscription(
          mTopicName, topic_type, rclcpp::QoS(QOS_QUEUE_SIZE),
          std::bind(&TopicBenchmarkComponent::topicCallback, this, _1),
          sub_opt);

        mSubMap[topic_type] = sub;
      }
    }
  }

  if (!mTopicAvailable) {
    RCLCPP_INFO_STREAM_ONCE(
      get_logger(), "Waiting for topic '"
        << mTopicName
        << "' to be published...");
  } else {
    if (mTopicTimer) {
      mTopicTimer->cancel();
    }
  }
}

void TopicBenchmarkComponent::topicCallback(
  std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  static bool first = true;

  // RCLCPP_INFO_STREAM(get_logger(), "Received a message of size: " <<
  // msg->size() );
  if (first) {
    mLastRecTime =
      std::chrono::high_resolution_clock::now();    // Set the start time point
    first = false;
    return;
  }

  auto now = std::chrono::high_resolution_clock::now();
  double elapsed_usec =
    std::chrono::duration_cast<std::chrono::microseconds>(now - mLastRecTime)
    .count();
  mLastRecTime = now;

  double freq = 1e6 / elapsed_usec;
  double avg_freq = mAvgFreq.addValue(freq);

  static double bw_scale = 8. / (1024. * 1024.);

  double bw = freq * bw_scale * msg->size();
  double bw_avg = avg_freq * bw_scale * msg->size();

  std::stringstream ss;
  if(!mUseRosLog) {
    ss << '\r';
  }
  ss << std::fixed << std::setprecision(2) << "#"
            << ++mTopicCount << " - Freq: " << freq << " Hz (Avg: " << avg_freq
            << " Hz) - Bandwidth: " << bw << " Mbps (Avg: " << bw_avg
            << " Mbps) - Msg size: " << msg->size() / (1024. * 1024.) << " MB";
  if(!mUseRosLog) {
    ss << std::flush;

    std::cout << ss.str();
  } else {
    RCLCPP_INFO_STREAM(get_logger(), ss.str());
  }

  // std::cout << " - Queue size: " << mAvgFreq.size() << std::endl;

  std::unique_ptr<zed_topic_benchmark_interfaces::msg::BenchmarkStatsStamped>
  stat_msg = std::make_unique<
    zed_topic_benchmark_interfaces::msg::BenchmarkStatsStamped>();

  stat_msg->header.stamp = get_clock()->now();
  stat_msg->topic_freq = freq;
  stat_msg->topic_avg_freq = avg_freq;
  stat_msg->topic_bw = bw;
  stat_msg->topic_avg_bw = bw_avg;

  mPub->publish(std::move(stat_msg));
}

}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::TopicBenchmarkComponent)
