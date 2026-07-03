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

#ifndef TOPIC_BENCHMARK_COMPONENT_HPP_
#define TOPIC_BENCHMARK_COMPONENT_HPP_

#include <rcutils/logging_macros.h>

#include <atomic>
#include <map>
#include <memory>
#include <rclcpp/generic_subscription.hpp>  // Not available before ROS 2 Humble
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <string>

#include "visibility_control.hpp"
#include "winavg.hpp"
#include <zed_topic_benchmark_interfaces/msg/benchmark_stats_stamped.hpp>

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

  /// @brief Check whether a test termination condition has been reached and,
  ///        if so, mark the test as complete and request shutdown.
  void checkTestCompletion();

  /// @brief Build the final statistics report and print it to the console
  ///        (and to the log file, if configured). Runs at most once.
  void generateReport();

private:
  rclcpp::TimerBase::SharedPtr mTopicTimer;
  rclcpp::TimerBase::SharedPtr mTestTimer;  ///< Enforces the duration limit
                                            ///< even if messages stop arriving

  // Parameters
  std::string mTopicName =
    DEFAULT_TOPIC_NAME;    ///< Name of the benchmarked topic
  int mWinSize = 500;      ///< Window size for frequency average
  bool mUseRosLog = false;  ///< Use ROS logging system
  double mTestDurationSec = 0.0;  ///< Test duration limit [s]. 0 = infinite
  int mTestSampleCount = 0;       ///< Test sample limit [#]. 0 = infinite
  std::string mLogFilePath = "";  ///< Path of the report log file. Empty = none

  std::atomic<bool> mTopicAvailable;  ///< Indicate if the benchmarked topic is
                                      ///< published by other nodes

  // Topic subscriptions
  std::map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> mSubMap;

  // Windowed averages computed on the message *periods* and *sizes* (not on
  // the instantaneous frequencies) to avoid the bias of averaging rates.
  WinAvg mPeriodAvg;  ///< Window average of the inter-arrival period [usec]
  WinAvg mSizeAvg;    ///< Window average of the serialized message size [bytes]

  // Time measuring (steady_clock is monotonic, unlike high_resolution_clock)
  bool mFirstMsg = true;  ///< Per-instance flag for the first received message
  std::chrono::steady_clock::time_point mLastRecTime;

  // Stats message publisher
  std::shared_ptr<rclcpp::Publisher<
      zed_topic_benchmark_interfaces::msg::BenchmarkStatsStamped>>
  mPub;
  uint64_t mTopicCount = 0;

  // ----> Aggregate statistics over the whole test (for the final report)
  std::mutex mStatsMux;
  std::string mFoundTopicType;
  std::chrono::steady_clock::time_point mTestStartTime;  ///< First message time
  std::chrono::steady_clock::time_point mTestLastTime;   ///< Last message time
  uint64_t mMsgCount = 0;        ///< Total number of received messages
  double mTotalBytes = 0.0;      ///< Sum of all received message sizes [bytes]
  double mMinFreq = 0.0;         ///< Minimum instantaneous frequency [Hz]
  double mMaxFreq = 0.0;         ///< Maximum instantaneous frequency [Hz]
  double mMinSizeBytes = 0.0;    ///< Minimum message size [bytes]
  double mMaxSizeBytes = 0.0;    ///< Maximum message size [bytes]
  double mMinBw = 0.0;           ///< Minimum instantaneous bandwidth [Mbps]
  double mMaxBw = 0.0;           ///< Maximum instantaneous bandwidth [Mbps]
  // <---- Aggregate statistics over the whole test

  std::atomic<bool> mTestComplete{false};     ///< A limit has been reached
  std::atomic<bool> mReportGenerated{false};  ///< The report has been printed
  std::string mStopReason = "interrupted by the user (Ctrl+C)";
};
}  // namespace stereolabs

#endif  // TOPIC_BENCHMARK_COMPONENT_HPP_
