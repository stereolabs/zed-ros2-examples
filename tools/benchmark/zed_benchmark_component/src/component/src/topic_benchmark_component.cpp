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

  // Start accounting CPU before any message is processed.
  mCpuMeter.start();

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
  mLatencyAvg.setNewSize(mWinSize);
  getParam("use_ros_log", mUseRosLog, mUseRosLog, "ROS Log: ");

  getParam(
    "subscription_mode", mSubscriptionMode, mSubscriptionMode,
    "Subscription mode: ");
  if (mSubscriptionMode != "auto" && mSubscriptionMode != "generic" &&
    mSubscriptionMode != "typed")
  {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Unknown 'subscription_mode' value '"
        << mSubscriptionMode << "'. Valid values are 'auto', 'generic' and "
        "'typed'. Falling back to 'auto'.");
    mSubscriptionMode = "auto";
  }

  // ----> Subscriber QoS
  // Declared here, and applied by passing the resulting QoS to the
  // subscription, so that the policies take effect on the generic path too.
  getParam(
    "qos.reliability", mQosReliability, mQosReliability,
    "QoS reliability: ");
  getParam("qos.durability", mQosDurability, mQosDurability, "QoS durability: ");
  getParam("qos.history", mQosHistory, mQosHistory, "QoS history: ");
  getParam("qos.depth", mQosDepth, mQosDepth, "QoS depth: ");
  // <---- Subscriber QoS

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

        subscribeToTopic(topic_type);
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

rclcpp::QoS TopicBenchmarkComponent::makeQos(
  const std::string & reliability, const std::string & durability,
  const std::string & history, int depth,
  std::vector<std::string> & warnings)
{
  if (depth < 1) {
    warnings.push_back(
      "'qos.depth' must be >= 1, got " + std::to_string(depth) + ". Using 1.");
    depth = 1;
  }

  // History decides how the QoS object is constructed, so resolve it first.
  std::string hist = history;
  if (hist != "keep_all" && hist != "keep_last") {
    warnings.push_back(
      "Unknown 'qos.history' value '" + hist +
      "'. Valid values are 'keep_last' and 'keep_all'. Using 'keep_last'.");
    hist = "keep_last";
  }
  rclcpp::QoS qos = (hist == "keep_all") ?
    rclcpp::QoS(rclcpp::KeepAll()) :
    rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(depth)));

  if (reliability == "reliable") {
    qos.reliable();
  } else {
    if (reliability != "best_effort") {
      warnings.push_back(
        "Unknown 'qos.reliability' value '" + reliability +
        "'. Valid values are 'best_effort' and 'reliable'. Using 'best_effort'.");
    }
    qos.best_effort();
  }

  if (durability == "transient_local") {
    qos.transient_local();
  } else {
    if (durability != "volatile") {
      warnings.push_back(
        "Unknown 'qos.durability' value '" + durability +
        "'. Valid values are 'volatile' and 'transient_local'. Using "
        "'volatile'.");
    }
    qos.durability_volatile();
  }

  return qos;
}

rclcpp::QoS TopicBenchmarkComponent::buildSubscriberQos()
{
  std::vector<std::string> warnings;
  const rclcpp::QoS qos = makeQos(
    mQosReliability, mQosDurability, mQosHistory, mQosDepth, warnings);
  for (const auto & w : warnings) {
    RCLCPP_WARN_STREAM(get_logger(), w);
  }

  // DDS only delivers when the request is no stronger than the offer, so these
  // two requests can silently yield no data at all. Say so up front rather than
  // leaving the user with an empty report and no explanation.
  //
  // Note on the ZED wrapper specifically: it publishes with
  // rclcpp::QoS(QOS_QUEUE_SIZE), QOS_QUEUE_SIZE being 10, i.e. the default
  // profile at depth 10 - Reliable, Volatile, KEEP_LAST(10). So `reliable` is
  // compatible with it, while `transient_local` is not.
  if (qos.reliability() == rclcpp::ReliabilityPolicy::Reliable) {
    RCLCPP_INFO(
      get_logger(),
      "Subscribing with RELIABLE reliability: this requires a Reliable "
      "publisher. Against a Best Effort publisher no message will arrive. "
      "Check the publisher with 'ros2 topic info -v <topic>'.");
  }
  if (qos.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
    RCLCPP_WARN(
      get_logger(),
      "Subscribing with TRANSIENT_LOCAL durability: this requires a Transient "
      "Local publisher. Most sensor publishers, including the ZED wrapper, are "
      "Volatile, in which case the QoS is incompatible and NO message will "
      "arrive. Check the publisher with 'ros2 topic info -v <topic>'.");
  }

  return qos;
}

std::string TopicBenchmarkComponent::qosToString(const rclcpp::QoS & qos)
{
  std::stringstream ss;
  switch (qos.reliability()) {
    case rclcpp::ReliabilityPolicy::Reliable: ss << "Reliable"; break;
    case rclcpp::ReliabilityPolicy::BestEffort: ss << "Best Effort"; break;
    case rclcpp::ReliabilityPolicy::SystemDefault: ss << "System Default"; break;
    default: ss << "Unknown"; break;
  }
  switch (qos.durability()) {
    case rclcpp::DurabilityPolicy::TransientLocal: ss << ", Transient Local";
      break;
    case rclcpp::DurabilityPolicy::Volatile: ss << ", Volatile"; break;
    case rclcpp::DurabilityPolicy::SystemDefault: ss << ", System Default";
      break;
    default: ss << ", Unknown"; break;
  }
  if (qos.history() == rclcpp::HistoryPolicy::KeepAll) {
    ss << ", KEEP_ALL";
  } else {
    ss << ", KEEP_LAST, depth " << qos.depth();
  }
  return ss.str();
}

void TopicBenchmarkComponent::subscribeToTopic(const std::string & topic_type)
{
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();

  // The QoS comes from this node's own `qos.*` parameters. Passing it as the
  // subscription's QoS argument is what makes the policies effective on every
  // path, including the generic one that ignores qos_overriding_options.
  const auto qos = buildSubscriberQos();

  const bool ipc_enabled = get_node_options().use_intra_process_comms();

  // Only a typed subscription can ever take the intra-process path, so "auto"
  // switches to it exactly when that path is available. Ordinary
  // separate-process runs keep the generic subscription and therefore keep
  // reporting exact wire bytes, as they always did.
  bool want_typed = false;
  if (mSubscriptionMode == "typed") {
    want_typed = true;
  } else if (mSubscriptionMode == "auto") {
    want_typed = ipc_enabled;
  }

  if (want_typed) {
    auto typed = createTypedSubscription(
      *this, mTopicName, topic_type, qos, sub_opt, ipc_enabled,
      [this](const Sample & sample) {this->recordSample(sample);});

    if (typed.sub) {
      mTypedSub = typed.sub;
      mZeroCopy = typed.zero_copy;
      mIntraProcessCapable = ipc_enabled;
      mSizeSemantics = SizeSemantics::MessageContent;
      mSubPathDesc = typed.description;
      // Read the QoS back from the subscription: on this path rclcpp may still
      // have applied a qos_overrides.* on top of the qos.* parameters, so the
      // requested values are not necessarily the granted ones.
      mActualQosDesc = qosToString(typed.sub->get_actual_qos());
      RCLCPP_INFO_STREAM(
        get_logger(), "Subscriber QoS: " << mActualQosDesc);

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Subscription path: " << mSubPathDesc
                              << (ipc_enabled ?
        " - intra-process enabled" :
        " - intra-process NOT enabled on this node"));
      if (mZeroCopy) {
        RCLCPP_INFO(
          get_logger(),
          "Zero-copy capable: if the publisher is in this process, its buffer "
          "is received by pointer with no serialization and no copy. The "
          "report confirms whether that actually happened.");
      } else if (ipc_enabled) {
        RCLCPP_INFO(
          get_logger(),
          "Intra-process capable: for publishers in this same process there is "
          "no serialization and no middleware, though rclcpp still copies the "
          "message into this subscription. A publisher in another process is "
          "still delivered through the middleware. Only the ZED type-adapted "
          "image path is truly zero-copy.");
      }
      return;
    }

    // No typed subscription exists for this type. Say so instead of silently
    // measuring something else than the user asked for.
    RCLCPP_WARN_STREAM(
      get_logger(),
      "No typed subscription is available for '"
        << topic_type
        << "', so the intra-process path cannot be used for this topic. "
        "Falling back to a generic subscription over the middleware.");
  } else if (ipc_enabled) {
    RCLCPP_WARN(
      get_logger(),
      "Intra-process communication is enabled but 'subscription_mode' is "
      "'generic': a generic (runtime-typed) subscription never takes the "
      "intra-process path in rclcpp, so this run measures the inter-process "
      "path. Use 'subscription_mode:=typed' to benchmark IPC.");
  }

  mSizeSemantics = SizeSemantics::SerializedWire;
  mSubPathDesc = "generic (runtime-typed)";
  mIntraProcessCapable = false;
  mZeroCopy = false;

  // rclcpp::create_generic_subscription() never reads
  // options.qos_overriding_options: it forwards the options to the
  // GenericSubscription constructor but never calls declare_qos_parameters(),
  // so no `qos_overrides.*` parameter is declared and any override the user
  // passed is silently discarded. True from Humble through Rolling. The QoS of
  // this path is therefore fixed at the values above.
  //
  // Silently ignoring an explicit instruction is the worst outcome, so say so.
  warnIfQosOverrideIgnored();

  auto generic_sub = create_generic_subscription(
    mTopicName, topic_type, qos,
    std::bind(&TopicBenchmarkComponent::topicCallback, this, _1),
    sub_opt);
  mActualQosDesc = qosToString(generic_sub->get_actual_qos());
  RCLCPP_INFO_STREAM(get_logger(), "Subscriber QoS: " << mActualQosDesc);
  mSubMap[topic_type] = generic_sub;
}

void TopicBenchmarkComponent::warnIfQosOverrideIgnored()
{
  // The parameter is never declared on this path, so it cannot be read back
  // with get_parameter(): look instead at the overrides the node was given.
  // NodeOptions::parameter_overrides() is NOT enough - it only holds what was
  // set programmatically, not what came from the command line. The parameters
  // interface merges both.
  const std::string prefix = "qos_overrides." + mTopicName + ".subscription.";
  std::vector<std::string> ignored;
  for (const auto & entry :
    get_node_parameters_interface()->get_parameter_overrides())
  {
    if (entry.first.rfind(prefix, 0) == 0) {
      ignored.push_back(entry.first);
    }
  }
  if (ignored.empty()) {
    return;
  }

  std::stringstream ss;
  for (size_t i = 0; i < ignored.size(); ++i) {
    ss << (i ? ", " : "") << ignored[i];
  }
  RCLCPP_WARN_STREAM(
    get_logger(),
    "The QoS override(s) " << ss.str() <<
      " will be IGNORED on this subscription path: rclcpp's "
      "create_generic_subscription() does not honour qos_overriding_options. "
      "Use this node's own parameters instead - qos.reliability, "
      "qos.durability, qos.history and qos.depth - which are applied on every "
      "subscription path.");
}

void TopicBenchmarkComponent::topicCallback(
  std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  // A generic subscription receives the serialized buffer, so its size is the
  // exact wire byte count. It carries no directly usable timestamp, hence no
  // latency measurement on this path.
  Sample sample;
  sample.size_bytes = msg->size();
  sample.has_stamp = false;
  recordSample(sample);
}

void TopicBenchmarkComponent::updateLatency(const Sample & sample)
{
  if (!sample.has_stamp) {
    return;
  }

  // End-to-end latency: publisher-side stamp to arrival in this callback. Both
  // sides must share the same clock, which is the case for a publisher and a
  // subscriber in the same ROS graph (and trivially so when composed).
  const double latency_msec =
    (get_clock()->now() - sample.stamp).nanoseconds() / 1e6;

  // A negative latency means the stamps are not comparable (e.g. the publisher
  // uses a different clock, or `use_sim_time` differs). Dropping the sample is
  // better than folding a meaningless value into the average.
  if (latency_msec < 0.0) {
    return;
  }

  mLatencyAvg.addValue(latency_msec);

  std::lock_guard<std::mutex> lock(mStatsMux);
  ++mLatencyCount;
  mTotalLatencyMsec += latency_msec;
  mMinLatencyMsec = (mLatencyCount == 1) ?
    latency_msec : std::min(mMinLatencyMsec, latency_msec);
  mMaxLatencyMsec = std::max(mMaxLatencyMsec, latency_msec);
}

void TopicBenchmarkComponent::recordSample(const Sample & sample)
{
  if (sample.intra_process_confirmed) {
    mIntraProcessObserved.store(true);
  }

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
      const double msg_size = static_cast<double>(sample.size_bytes);
      mTestStartTime = now;
      mTestLastTime = now;
      mMsgCount = 1;
      mTotalBytes = msg_size;
      mMinSizeBytes = mMaxSizeBytes = msg_size;
    }
    updateLatency(sample);
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

  const double msg_size = static_cast<double>(sample.size_bytes);

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
  updateLatency(sample);

  std::stringstream ss;
  ss << std::fixed << std::setprecision(2)
     << "#" << std::setw(6) << std::left << ++mTopicCount << std::right
     << " | Freq " << std::setw(7) << freq << "/" << std::setw(7) << avg_freq
     << " Hz | BW " << std::setw(7) << bw << "/" << std::setw(7) << bw_avg
     << " Mbps | " << std::setw(9) << humanReadableSize(msg_size);
  if (sample.has_stamp) {
    ss << " | Lat " << std::setw(6) << mLatencyAvg.getAvg() << " ms";
  }

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
  if (sample.has_stamp) {
    stat_msg->topic_latency =
      (stat_msg->header.stamp.sec == 0 && stat_msg->header.stamp.nanosec == 0) ?
      0.0F :
      static_cast<float>(
      (rclcpp::Time(stat_msg->header.stamp) - sample.stamp).nanoseconds() / 1e6);
    stat_msg->topic_avg_latency = static_cast<float>(mLatencyAvg.getAvg());
  } else {
    stat_msg->topic_latency = 0.0F;
    stat_msg->topic_avg_latency = 0.0F;
  }

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

  // Read the CPU counters before taking the lock: /proc access should not be
  // done while holding the statistics mutex the callbacks contend on.
  mCpuSeconds = mCpuMeter.cpuSeconds();

  std::lock_guard<std::mutex> lock(mStatsMux);

  constexpr double MB = 1024. * 1024.;

  std::stringstream rep;
  rep << "\n";
  rep << "================ ZED TOPIC BENCHMARK REPORT ================\n";
  rep << "Topic name:        " << mTopicName << "\n";
  rep << "Topic type:        "
      << (mFoundTopicType.empty() ? "N/A" : mFoundTopicType) << "\n";
  // Always state which path was measured and how the sizes must be read: a
  // bandwidth over wire bytes and one over message content are different
  // quantities, and an inter-process figure must never be mistaken for an
  // intra-process one.
  // If no subscription was ever created the topic was never seen, so there is
  // no path to describe. Printing the default would assert a subscription that
  // never existed.
  const bool subscribed = !mSubMap.empty() || mTypedSub != nullptr;
  if (!subscribed) {
    rep << "Subscription:      none - the topic was never seen\n";
    rep << "Delivery path:     n/a\n";
    rep << "Stop reason:       " << mStopReason << "\n";
    rep << "No message received: no statistics available.\n";
    rep << "===========================================================\n";
    if (mUseRosLog && rclcpp::ok()) {
      RCLCPP_INFO_STREAM(get_logger(), rep.str());
    } else {
      std::cout << "\n" << rep.str() << std::endl;
    }
    if (!mLogFilePath.empty()) {
      std::ofstream ofs(mLogFilePath, std::ios::out | std::ios::trunc);
      if (ofs.is_open()) {
        ofs << rep.str();
      }
    }
    return;
  }

  rep << "Subscription:      " << mSubPathDesc << "\n";
  rep << "Delivery path:     ";
  if (mIntraProcessObserved.load()) {
    // Only provable on the type-adapted path, where the custom C++ type that
    // arrived has no wire representation at all.
    rep << "intra-process, zero-copy - CONFIRMED (publisher's buffer received "
      "by pointer)";
  } else if (mIntraProcessCapable) {
    // Deliberately stated as a capability. Enabling intra-process comms on this
    // node says nothing about where the publisher lives, and claiming
    // intra-process delivery here would be exactly the kind of unfounded
    // assertion this report exists to avoid.
    rep << "intra-process capable, NOT confirmed - a typed subscription with "
      "intra-process comms enabled takes that path only for publishers in "
      "this same process";
  } else if (mZeroCopy) {
    rep << "type-adapted subscription created, but no message was received "
      "through it";
  } else if (mSizeSemantics == SizeSemantics::SerializedWire) {
    rep << "inter-process (middleware) - a generic subscription can never take "
      "the intra-process path";
  } else {
    rep << "inter-process (middleware) - intra-process comms not enabled on "
      "this node";
  }
  rep << "\n";
  rep << "Size semantics:    "
      << (mSizeSemantics == SizeSemantics::SerializedWire ?
  "serialized wire bytes" :
  "message content bytes (no CDR framing) - NOT comparable to wire bytes")
      << "\n";
  // The QoS is part of the measurement's provenance: a Reliable subscriber and
  // a Best Effort one can see very different rates on a lossy link. This is the
  // QoS the middleware actually granted, read back from the subscription.
  rep << "Subscriber QoS:    "
      << (mActualQosDesc.empty() ? "n/a" : mActualQosDesc) << "\n";
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
    mCpuPercent = (elapsed_sec > 0.0) ? 100.0 * mCpuSeconds / elapsed_sec : 0.0;
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

    // Latency is the figure that actually shows what the intra-process path
    // buys: on that path nothing is transported, so "bandwidth" is notional.
    if (mLatencyCount > 0) {
      const double mean_latency = mTotalLatencyMsec / mLatencyCount;
      rep << "Latency [ms]     - mean: " << mean_latency
          << " | min: " << mMinLatencyMsec
          << " | max: " << mMaxLatencyMsec
          << " (" << mLatencyCount << " samples)\n";
    } else {
      rep << "Latency [ms]     - not available on this subscription path\n";
    }

    if (mCpuMeter.valid()) {
      // Extra precision: a light topic can consume only a few milliseconds of
      // CPU over the whole test, which two decimals would flatten to "0.00".
      rep << "Process CPU:       " << std::setprecision(3) << mCpuSeconds
          << " s (" << std::setprecision(2) << mCpuPercent
          << "% of one core)\n";
      // Scope matters: composed runs share the process with the publisher, so
      // this figure is only comparable across runs when the reader knows what
      // it covers. Total CPU across every process involved is the fair metric.
      rep << "                   whole process, including any other component "
        "loaded in it\n";
    }
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
