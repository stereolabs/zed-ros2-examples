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

#include "topic_benchmark_component.hpp"

#include <rcutils/logging_macros.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_overriding_options.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <sstream>

using namespace std::placeholders;

namespace stereolabs
{

const int QOS_QUEUE_SIZE = 1;

namespace
{
// Format a byte count with adaptive units (B, KB, MB, GB) so that small
// messages (e.g. IMU samples of a few hundred bytes) are not rounded to
// "0.00 MB".
std::string humanReadableSize(double bytes)
{
  const char * units[] = {"B", "KB", "MB", "GB", "TB"};
  size_t unit = 0;
  double value = bytes;
  while (value >= 1024.0 && unit < (sizeof(units) / sizeof(units[0]) - 1)) {
    value /= 1024.0;
    ++unit;
  }
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << value << " " << units[unit];
  return ss.str();
}
}  // namespace

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

  // Make sure the final report is produced when the process is shut down,
  // either because a test limit has been reached or because the user pressed
  // Ctrl+C. The report is generated at most once (see generateReport()).
  rclcpp::on_shutdown([this]() {this->generateReport();});
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

  // When a duration limit is set, a periodic timer enforces it even if the
  // benchmarked topic stops publishing before the limit is reached.
  if (mTestDurationSec > 0.0) {
    mTestTimer = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&TopicBenchmarkComponent::checkTestCompletion, this));
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
      "topic_name:=/zed2i/zed_node/rgb/color/rect/image'");
  }
  getParam("avg_win_size", mWinSize, mWinSize, "Average window size: ");
  mPeriodAvg.setNewSize(mWinSize);
  mSizeAvg.setNewSize(mWinSize);
  getParam("use_ros_log", mUseRosLog, mUseRosLog, "ROS Log: ");

  getParam(
    "test_duration_sec", mTestDurationSec, mTestDurationSec,
    "Test duration [sec] (0 = infinite): ");
  getParam(
    "test_sample_count", mTestSampleCount, mTestSampleCount,
    "Test sample count [#] (0 = infinite): ");
  getParam("log_file_path", mLogFilePath, mLogFilePath, "Report log file: ");
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
        {
          std::lock_guard<std::mutex> lock(mStatsMux);
          mFoundTopicType = topic_type;
        }
        RCLCPP_INFO_STREAM(
          get_logger(), "Found topic: '" << mTopicName
                                         << "' of type: '"
                                         << topic_type << "'");

        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.qos_overriding_options =
          rclcpp::QosOverridingOptions::with_default_policies();

        // Subscribe with Best Effort reliability by default: a Best Effort
        // subscriber is compatible with both Reliable and Best Effort
        // publishers, while a Reliable subscriber cannot connect to a Best
        // Effort publisher (common for sensor data such as images and point
        // clouds). The reliability can still be overridden at runtime via the
        // `qos_overrides` parameters.
        std::shared_ptr<rclcpp::GenericSubscription> sub =
          create_generic_subscription(
          mTopicName, topic_type, rclcpp::QoS(QOS_QUEUE_SIZE).best_effort(),
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
  auto now = std::chrono::steady_clock::now();

  if (mTestComplete.load()) {
    // A test limit has already been reached: ignore any further message so the
    // report reflects exactly the requested duration/sample count.
    return;
  }

  // The very first message only initializes the time reference: there is no
  // interval yet to derive a frequency from. `mFirstMsg` is a per-instance
  // member (not a function-local static) so that multiple benchmark
  // components composed in the same process do not share this state.
  if (mFirstMsg) {
    mLastRecTime = now;
    mFirstMsg = false;

    {
      std::lock_guard<std::mutex> lock(mStatsMux);
      const double msg_size = static_cast<double>(msg->size());
      mTestStartTime = now;
      mTestLastTime = now;
      mMsgCount = 1;
      mTotalBytes = msg_size;
      mMinSizeBytes = mMaxSizeBytes = msg_size;
    }
    checkTestCompletion();
    return;
  }

  // Use nanosecond resolution then scale, to avoid truncating sub-usec
  // intervals to zero on high-rate topics.
  double elapsed_usec =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now - mLastRecTime)
    .count() / 1e3;
  mLastRecTime = now;

  if (elapsed_usec <= 0.0) {
    // Unmeasurable interval (two messages in the same tick): skip this sample
    // to avoid a division by zero / infinite frequency.
    return;
  }

  const double msg_size = static_cast<double>(msg->size());

  // Instantaneous values from the last inter-arrival interval.
  double freq = 1e6 / elapsed_usec;

  // Windowed averages are computed on the periods and sizes, NOT on the
  // instantaneous frequencies. Averaging instantaneous rates (mean of 1/dt)
  // is biased high by Jensen's inequality and overestimates the true rate.
  // The correct windowed mean frequency is 1 / mean(dt) and the correct
  // windowed bandwidth is mean(size) / mean(dt) (= total bits / total time),
  // matching the behavior of `ros2 topic hz`.
  double avg_period_usec = mPeriodAvg.addValue(elapsed_usec);
  double avg_size = mSizeAvg.addValue(msg_size);
  double avg_freq = (avg_period_usec > 0.0) ? 1e6 / avg_period_usec : 0.0;

  constexpr double bw_scale = 8. / (1024. * 1024.);

  double bw = freq * bw_scale * msg_size;
  double bw_avg = avg_freq * bw_scale * avg_size;

  // The min/max statistics track the *windowed average* rate, not the raw
  // single-sample instantaneous rate: a single short inter-arrival interval
  // (e.g. two messages delivered back-to-back by the executor, or a publisher
  // burst) yields a huge instantaneous frequency that is not representative.
  // They are recorded only once the averaging window is full, so the start-up
  // transient does not pollute the extremes.
  bool window_full = mPeriodAvg.size() >= static_cast<size_t>(mWinSize);

  // ----> Update the aggregate statistics used by the final report
  {
    std::lock_guard<std::mutex> lock(mStatsMux);
    mTestLastTime = now;
    ++mMsgCount;
    mTotalBytes += msg_size;
    mMinSizeBytes = std::min(mMinSizeBytes, msg_size);
    mMaxSizeBytes = std::max(mMaxSizeBytes, msg_size);
    if (window_full) {
      mMinFreq = (mMinFreq == 0.0) ? avg_freq : std::min(mMinFreq, avg_freq);
      mMaxFreq = std::max(mMaxFreq, avg_freq);
      mMinBw = (mMinBw == 0.0) ? bw_avg : std::min(mMinBw, bw_avg);
      mMaxBw = std::max(mMaxBw, bw_avg);
    }
  }
  // <---- Update the aggregate statistics used by the final report

  // Compact, fixed-width line. Values are shown as "instant/average". The
  // layout is kept under 80 columns so it does not wrap on a default terminal:
  // a wrapped line would break the in-place (\r) update and flood the console.
  // Fixed-width fields also keep the columns from shifting as values change.
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2)
     << "#" << std::setw(6) << std::left << ++mTopicCount << std::right
     << " | Freq " << std::setw(7) << freq << "/" << std::setw(7) << avg_freq
     << " Hz | BW " << std::setw(7) << bw << "/" << std::setw(7) << bw_avg
     << " Mbps | " << std::setw(9) << humanReadableSize(msg_size);

  if (!mUseRosLog) {
    // '\r' rewinds to the start of the line; '\033[K' erases anything left
    // over from a previously longer line.
    std::cout << '\r' << ss.str() << "\033[K" << std::flush;
  } else {
    RCLCPP_INFO_STREAM(get_logger(), ss.str());
  }

  // std::cout << " - Queue size: " << mPeriodAvg.size() << std::endl;

  std::unique_ptr<zed_topic_benchmark_interfaces::msg::BenchmarkStatsStamped>
  stat_msg = std::make_unique<
    zed_topic_benchmark_interfaces::msg::BenchmarkStatsStamped>();

  stat_msg->header.stamp = get_clock()->now();
  stat_msg->topic_freq = freq;
  stat_msg->topic_avg_freq = avg_freq;
  stat_msg->topic_bw = bw;
  stat_msg->topic_avg_bw = bw_avg;

  mPub->publish(std::move(stat_msg));

  checkTestCompletion();
}

void TopicBenchmarkComponent::checkTestCompletion()
{
  // Must be called WITHOUT holding mStatsMux: on completion it triggers the
  // shutdown callback, which locks mStatsMux to build the report.
  if (mTestComplete.load()) {
    return;
  }

  bool by_samples =
    (mTestSampleCount > 0) &&
    (mMsgCount >= static_cast<uint64_t>(mTestSampleCount));

  bool by_duration = false;
  if (mTestDurationSec > 0.0 && mMsgCount > 0) {
    double elapsed_sec =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - mTestStartTime)
      .count() / 1e9;
    by_duration = elapsed_sec >= mTestDurationSec;
  }

  if (!by_samples && !by_duration) {
    return;
  }

  if (mTestComplete.exchange(true)) {
    return;  // another condition already triggered completion
  }

  mStopReason = by_samples ?
    "test completed (sample count reached)" :
    "test completed (duration reached)";

  if (mTestTimer) {
    mTestTimer->cancel();
  }

  RCLCPP_INFO_STREAM(get_logger(), "\n*** " << mStopReason << " ***");

  // Generate the report now, while the context is still valid (clean logging),
  // then request shutdown to stop the executor. The on_shutdown callback will
  // find the report already generated and skip it.
  generateReport();
  rclcpp::shutdown();
}

void TopicBenchmarkComponent::generateReport()
{
  if (mReportGenerated.exchange(true)) {
    return;  // generate the report at most once
  }

  std::lock_guard<std::mutex> lock(mStatsMux);

  constexpr double MB = 1024. * 1024.;

  std::stringstream rep;
  rep << "\n";
  rep << "================ ZED TOPIC BENCHMARK REPORT ================\n";
  rep << "Topic name:        " << mTopicName << "\n";
  rep << "Topic type:        "
      << (mFoundTopicType.empty() ? "N/A" : mFoundTopicType) << "\n";
  rep << "Stop reason:       " << mStopReason << "\n";

  if (mMsgCount == 0) {
    rep << "No message received: no statistics available.\n";
    rep << "===========================================================\n";
  } else {
    double elapsed_sec =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      mTestLastTime - mTestStartTime)
      .count() / 1e9;
    uint64_t intervals = (mMsgCount > 1) ? (mMsgCount - 1) : 0;
    double mean_freq = (elapsed_sec > 0.0) ? intervals / elapsed_sec : 0.0;
    double mean_size = mTotalBytes / mMsgCount;
    double mean_bw =
      (elapsed_sec > 0.0) ? (mTotalBytes * 8.0 / MB) / elapsed_sec : 0.0;

    // If the averaging window never filled (very short test) the windowed
    // min/max were never recorded: fall back to the overall means so the
    // report shows representative numbers instead of zeros.
    if (mMaxFreq == 0.0) {
      mMinFreq = mMaxFreq = mean_freq;
      mMinBw = mMaxBw = mean_bw;
    }

    rep << std::fixed << std::setprecision(2);
    rep << "Test duration:     " << elapsed_sec << " s\n";
    rep << "Messages received: " << mMsgCount << "\n";
    rep << "-----------------------------------------------------------\n";
    rep << "Frequency [Hz]   - mean: " << mean_freq
        << " | min: " << mMinFreq << " | max: " << mMaxFreq << "\n";
    rep << "Msg size         - mean: " << humanReadableSize(mean_size)
        << " | min: " << humanReadableSize(mMinSizeBytes)
        << " | max: " << humanReadableSize(mMaxSizeBytes) << "\n";
    rep << "Bandwidth [Mbps] - mean: " << mean_bw
        << " | min: " << mMinBw << " | max: " << mMaxBw << "\n";
    rep << "Total data:        " << humanReadableSize(mTotalBytes) << "\n";
    rep << "===========================================================\n";
  }

  // Print the report to the console. Use the ROS logger only when the context
  // is still valid (e.g. test completed): during a Ctrl+C shutdown the rosout
  // publisher is already gone, so fall back to stdout to avoid a noisy error.
  if (mUseRosLog && rclcpp::ok()) {
    RCLCPP_INFO_STREAM(get_logger(), rep.str());
  } else {
    std::cout << "\n" << rep.str() << std::endl;
  }

  // Optionally write the same report to a log file.
  if (!mLogFilePath.empty()) {
    std::ofstream ofs(mLogFilePath, std::ios::out | std::ios::trunc);
    if (ofs.is_open()) {
      ofs << rep.str();
      ofs.close();
      RCLCPP_INFO_STREAM(
        get_logger(), "Benchmark report written to: " << mLogFilePath);
    } else {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Unable to write the benchmark report to: " << mLogFilePath);
    }
  }
}

}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::TopicBenchmarkComponent)
