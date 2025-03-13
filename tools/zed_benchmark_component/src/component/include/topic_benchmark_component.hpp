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

#ifndef TOPIC_BENCHMARK_COMPONENT_HPP_
#define TOPIC_BENCHMARK_COMPONENT_HPP_

#include <rcutils/logging_macros.h>

#include <atomic>
#include <map>
#include <memory>
#include <rclcpp/generic_subscription.hpp>  // Not available before ROS2 Humble
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <string>

#include "visibility_control.hpp"
#include "winavg.hpp"
#include "zed_topic_benchmark_interfaces/msg/benchmark_stats_stamped.hpp"

#define DEFAULT_TOPIC_NAME std::string("topic_name")

namespace stereolabs
{
class TopicBenchmarkComponent : public rclcpp::Node
{
public:
  TOPIC_BENCHMARK_PUBLIC
  explicit TopicBenchmarkComponent(const rclcpp::NodeOptions & options);
  virtual ~TopicBenchmarkComponent();

protected:
  void init();

  // ----> Node Parameters
  template<typename T>
  void getParam(
    std::string paramName, T defValue, T & outVal,
    std::string log_info = std::string(), bool dynamic = false);

  void getParameters();
  // ----> Node Parameters

  void updateTopicInfo();  ///< Update the information to subscribe to the topic
                           ///< under benchmarking

  void topicCallback(std::shared_ptr<rclcpp::SerializedMessage> msg);

private:
  double mSubFreqTot;  ///< Total of subscriber receiving frequency for average
                       ///< computation
  double mSubFreqBw;   ///< Average topic bandwidth (topic_size x avg_freq)

  rclcpp::TimerBase::SharedPtr mTopicTimer;

  // Parameters
  std::string mTopicName =
    DEFAULT_TOPIC_NAME;    ///< Name of the benchmarked topic
  int mWinSize = 500;      ///< Window size for frequency average
  bool mUseRosLog = false;  ///< Use ROS logging system

  std::atomic<bool> mTopicAvailable;  ///< Indicate if the benchmarked topic is
                                      ///< published by other nodes

  // Topic subscriptions
  std::map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> mSubMap;

  // Average values
  WinAvg mAvgFreq;

  // Time measuring
  std::chrono::high_resolution_clock::time_point mLastRecTime;

  // Stats message publisher
  std::shared_ptr<rclcpp::Publisher<
      zed_topic_benchmark_interfaces::msg::BenchmarkStatsStamped>>
  mPub;
  uint64_t mTopicCount = 0;
};
}  // namespace stereolabs

#endif  // TOPIC_BENCHMARK_COMPONENT_HPP_
