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

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "topic_benchmark_component.hpp"

using stereolabs::TopicBenchmarkComponent;

namespace
{
rclcpp::QoS make(
  const std::string & rel, const std::string & dur, const std::string & hist,
  int depth, std::vector<std::string> & warn)
{
  return TopicBenchmarkComponent::makeQos(rel, dur, hist, depth, warn);
}
}  // namespace

TEST(QosTest, DefaultsReproduceTheHistoricalQos) {
  std::vector<std::string> warn;
  const auto qos = make("best_effort", "volatile", "keep_last", 1, warn);
  EXPECT_TRUE(warn.empty());
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
  EXPECT_EQ(qos.history(), rclcpp::HistoryPolicy::KeepLast);
  EXPECT_EQ(qos.depth(), 1u);
}

TEST(QosTest, EveryPolicyCanBeOverridden) {
  std::vector<std::string> warn;
  const auto qos = make("reliable", "transient_local", "keep_last", 25, warn);
  EXPECT_TRUE(warn.empty());
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
  EXPECT_EQ(qos.history(), rclcpp::HistoryPolicy::KeepLast);
  EXPECT_EQ(qos.depth(), 25u);
}

TEST(QosTest, KeepAllIsHonoured) {
  std::vector<std::string> warn;
  const auto qos = make("reliable", "volatile", "keep_all", 1, warn);
  EXPECT_TRUE(warn.empty());
  EXPECT_EQ(qos.history(), rclcpp::HistoryPolicy::KeepAll);
}

// A typo must not be applied as if it were valid, and must not be silent
// either: that is the failure mode this whole area exists to avoid.
TEST(QosTest, UnknownValuesFallBackAndWarn) {
  std::vector<std::string> warn;
  const auto qos = make("Reliable", "durable", "keep_latest", 1, warn);
  EXPECT_EQ(warn.size(), 3u) << "each bad value must produce one warning";
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
  EXPECT_EQ(qos.history(), rclcpp::HistoryPolicy::KeepLast);
}

TEST(QosTest, NonPositiveDepthIsClampedAndWarned) {
  for (int bad : {0, -1, -100}) {
    std::vector<std::string> warn;
    const auto qos = make("best_effort", "volatile", "keep_last", bad, warn);
    EXPECT_EQ(warn.size(), 1u) << "depth " << bad;
    EXPECT_EQ(qos.depth(), 1u) << "depth " << bad;
  }
}

TEST(QosTest, StringFormIsReadable) {
  std::vector<std::string> warn;
  EXPECT_EQ(
    TopicBenchmarkComponent::qosToString(
      make("reliable", "transient_local", "keep_last", 7, warn)),
    "Reliable, Transient Local, KEEP_LAST, depth 7");
  EXPECT_EQ(
    TopicBenchmarkComponent::qosToString(
      make("best_effort", "volatile", "keep_all", 1, warn)),
    "Best Effort, Volatile, KEEP_ALL");
}
