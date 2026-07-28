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

#include "cpu_meter.hpp"
#include "typed_subscriber.hpp"
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

  /// @brief Map the `qos.*` parameter strings onto an rclcpp::QoS.
  ///
  /// Pure so it can be unit tested: any unrecognised value is replaced by the
  /// default and appended to `warnings` rather than silently ignored.
  /// @param[out] warnings One message per input that had to be corrected.
  TOPIC_BENCHMARK_PUBLIC
  static rclcpp::QoS makeQos(
    const std::string & reliability, const std::string & durability,
    const std::string & history, int depth,
    std::vector<std::string> & warnings);

  /// @brief Human-readable form of a QoS, for logs and the report.
  TOPIC_BENCHMARK_PUBLIC
  static std::string qosToString(const rclcpp::QoS & qos);

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

  /// @brief Subscribe to `topic_type` on the benchmarked topic, choosing the
  ///        generic or the typed path according to `subscription_mode`.
  void subscribeToTopic(const std::string & topic_type);

  void topicCallback(std::shared_ptr<rclcpp::SerializedMessage> msg);

  /// @brief Single entry point for every measurement, whichever subscription
  ///        path produced it. Keeps one statistics engine for all paths.
  void recordSample(const Sample & sample);

  /// @brief Fold the end-to-end latency of `sample` into the statistics, when
  ///        the sample carries a usable publisher-side timestamp.
  void updateLatency(const Sample & sample);

  /// @brief Warn when the user passed a `qos_overrides.*` for the benchmarked
  ///        topic that the generic subscription path cannot honour.
  void warnIfQosOverrideIgnored();

  /// @brief Build the subscriber QoS from the `qos.*` parameters.
  ///
  /// These are declared by this node rather than relying on rclcpp's
  /// `qos_overrides.*` mechanism, because that mechanism is not applied to a
  /// generic subscription. The QoS *argument* to create_generic_subscription()
  /// is honoured, so setting it explicitly is what makes the policies
  /// effective on every subscription path.
  rclcpp::QoS buildSubscriberQos();

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

  /// Which subscription path to use: "auto", "generic" or "typed".
  /// - "generic": always a runtime-typed rclcpp::GenericSubscription. Works for
  ///   any message type and reports exact wire bytes, but can never take the
  ///   intra-process path.
  /// - "typed": a compile-time typed subscription, which is the only kind that
  ///   rclcpp registers with the IntraProcessManager. Required to benchmark IPC.
  /// - "auto": "typed" when intra-process comms are enabled on this node and the
  ///   topic type is supported, "generic" otherwise. This keeps the historical
  ///   wire-accurate behaviour for ordinary separate-process runs.
  std::string mSubscriptionMode = "auto";

  // ----> Subscriber QoS parameters
  // Defaults reproduce the historical QoS: a Best Effort subscriber is
  // compatible with both Reliable and Best Effort publishers, so it matches
  // sensor-data topics out of the box.
  std::string mQosReliability = "best_effort";  ///< best_effort | reliable
  std::string mQosDurability = "volatile";      ///< volatile | transient_local
  std::string mQosHistory = "keep_last";        ///< keep_last | keep_all
  int mQosDepth = 1;                            ///< depth, for keep_last
  /// The QoS actually granted by the middleware, read back from the
  /// subscription. Reported instead of the requested values, so the report
  /// cannot claim a policy that was never applied.
  std::string mActualQosDesc;
  // <---- Subscriber QoS parameters

  std::atomic<bool> mTopicAvailable;  ///< Indicate if the benchmarked topic is
                                      ///< published by other nodes

  // Topic subscriptions. Only one of the two is populated, depending on the
  // resolved subscription path.
  std::map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> mSubMap;
  rclcpp::SubscriptionBase::SharedPtr mTypedSub;

  // Windowed averages computed on the message *periods* and *sizes* (not on
  // the instantaneous frequencies) to avoid the bias of averaging rates.
  WinAvg mPeriodAvg;  ///< Window average of the inter-arrival period [usec]
  WinAvg mSizeAvg;    ///< Window average of the message size [bytes]
  WinAvg mLatencyAvg;  ///< Window average of the end-to-end latency [msec]

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

  // End-to-end latency: publisher-side header stamp to arrival in the callback.
  // This, not bandwidth, is the figure that reveals what the intra-process path
  // actually buys, because on that path no bytes are transported at all.
  uint64_t mLatencyCount = 0;      ///< Messages carrying a usable stamp
  double mTotalLatencyMsec = 0.0;  ///< Sum of the latencies [msec]
  double mMinLatencyMsec = 0.0;    ///< Minimum latency [msec]
  double mMaxLatencyMsec = 0.0;    ///< Maximum latency [msec]
  // <---- Aggregate statistics over the whole test

  // ----> Resolved subscription path, for logs and for the report
  /// How to read the reported sizes. Set when the subscription is created.
  SizeSemantics mSizeSemantics = SizeSemantics::SerializedWire;
  std::string mSubPathDesc = "generic (runtime-typed)";
  /// Typed subscription on a node with intra-process comms enabled. This is a
  /// *capability*: delivery is intra-process only for publishers living in this
  /// same process, which cannot be checked from inside the callback.
  bool mIntraProcessCapable = false;
  bool mZeroCopy = false;  ///< ZED type-adapted subscription in use
  /// Set once a message has actually been delivered intra-process. Only the
  /// type-adapted path can prove this, so it is the sole claim the report makes
  /// about observed - as opposed to merely possible - intra-process delivery.
  std::atomic<bool> mIntraProcessObserved{false};
  // <---- Resolved subscription path

  CpuMeter mCpuMeter;      ///< Process CPU consumed over the test
  double mCpuSeconds = 0.0;  ///< CPU seconds, captured when the test ends
  double mCpuPercent = 0.0;  ///< CPU percent of one core, at test end

  std::atomic<bool> mTestComplete{false};     ///< A limit has been reached
  std::atomic<bool> mReportGenerated{false};  ///< The report has been printed
  std::string mStopReason = "interrupted by the user (Ctrl+C)";
};
}  // namespace stereolabs

#endif  // TOPIC_BENCHMARK_COMPONENT_HPP_
