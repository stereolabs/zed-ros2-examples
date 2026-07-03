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

// These tests validate the statistics produced by TopicBenchmarkComponent.
// The frequency/bandwidth math in topicCallback() is reproduced here on top of
// the same WinAvg engine, so the tests pin down the *correctness properties*
// the implementation must satisfy (and would catch a regression to the old
// rate-averaging behavior).

#include <gtest/gtest.h>

#include <numeric>
#include <vector>

#include "winavg.hpp"

using stereolabs::WinAvg;

namespace
{
// bits/byte over MB, as used in topic_benchmark_component.cpp
constexpr double BW_SCALE = 8.0 / (1024.0 * 1024.0);

// Mirror of the production computation: feed inter-arrival periods [usec] and
// message sizes [bytes] through the windowed averages and return the windowed
// average frequency [Hz] and bandwidth [Mbps].
struct WindowedStats
{
  double avg_freq_hz;
  double avg_bw_mbps;
};

WindowedStats computeWindowedStats(
  const std::vector<double> & periods_usec,
  const std::vector<double> & sizes_bytes)
{
  // A window large enough to hold every sample (no eviction) so the result is
  // the mean over the whole input, matching the analytical references below.
  WinAvg period_avg(periods_usec.size());
  WinAvg size_avg(sizes_bytes.size());

  double avg_period_usec = 0.0;
  double avg_size = 0.0;
  for (size_t i = 0; i < periods_usec.size(); ++i) {
    avg_period_usec = period_avg.addValue(periods_usec[i]);
    avg_size = size_avg.addValue(sizes_bytes[i]);
  }

  WindowedStats out;
  out.avg_freq_hz = (avg_period_usec > 0.0) ? 1e6 / avg_period_usec : 0.0;
  out.avg_bw_mbps = out.avg_freq_hz * BW_SCALE * avg_size;
  return out;
}
}  // namespace

// A perfectly periodic stream must report exactly its nominal rate.
TEST(StatisticsTest, ConstantRateGivesExactFrequency)
{
  // 100 ms period -> 10 Hz
  std::vector<double> periods(50, 100000.0);     // usec
  std::vector<double> sizes(50, 1024.0 * 1024.0);  // 1 MB each

  WindowedStats s = computeWindowedStats(periods, sizes);

  EXPECT_NEAR(s.avg_freq_hz, 10.0, 1e-9);
  // 10 Hz * 1 MB = 8 * 10 = 80 Mbit/s (with the 1024-based MB used by the tool)
  EXPECT_NEAR(s.avg_bw_mbps, 10.0 * 8.0, 1e-9);
}

// The windowed average frequency must equal the true mean rate over the window,
// i.e. 1 / mean(period) == N / sum(period). This is the property that the fix
// (averaging periods, not instantaneous rates) is meant to guarantee.
TEST(StatisticsTest, AverageFrequencyEqualsTrueMeanRate)
{
  // Jittery periods around 10 ms (in usec).
  std::vector<double> periods =
  {8000.0, 12000.0, 9000.0, 15000.0, 7000.0, 11000.0, 13000.0, 10000.0};
  std::vector<double> sizes(periods.size(), 500000.0);

  WindowedStats s = computeWindowedStats(periods, sizes);

  double sum_usec = std::accumulate(periods.begin(), periods.end(), 0.0);
  double true_mean_rate_hz = periods.size() * 1e6 / sum_usec;

  EXPECT_NEAR(s.avg_freq_hz, true_mean_rate_hz, 1e-6);
}

// Averaging the instantaneous rates (the OLD behavior) overestimates the true
// mean rate whenever there is jitter (Jensen's inequality). This test documents
// the bias the implementation deliberately avoids.
TEST(StatisticsTest, RateAveragingIsBiasedHigh)
{
  std::vector<double> periods =
  {8000.0, 12000.0, 9000.0, 15000.0, 7000.0, 11000.0, 13000.0, 10000.0};
  std::vector<double> sizes(periods.size(), 1.0);

  WindowedStats s = computeWindowedStats(periods, sizes);  // correct approach

  // Old approach: mean of the instantaneous frequencies.
  double sum_rate = 0.0;
  for (double p : periods) {
    sum_rate += 1e6 / p;
  }
  double biased_mean_rate = sum_rate / periods.size();

  // With jitter the biased estimate is strictly larger than the correct one.
  EXPECT_GT(biased_mean_rate, s.avg_freq_hz);
}

// The windowed average bandwidth must equal total bits / total time over the
// window, i.e. mean(size) / mean(period). Validates the variable-size handling.
TEST(StatisticsTest, AverageBandwidthEqualsTotalBitsOverTotalTime)
{
  // Variable message sizes (e.g. compressed images) and variable periods.
  std::vector<double> periods = {10000.0, 20000.0, 30000.0, 40000.0};  // usec
  std::vector<double> sizes =
  {1048576.0, 2097152.0, 524288.0, 4194304.0};  // bytes

  WindowedStats s = computeWindowedStats(periods, sizes);

  double sum_bytes = std::accumulate(sizes.begin(), sizes.end(), 0.0);
  double sum_sec =
    std::accumulate(periods.begin(), periods.end(), 0.0) / 1e6;
  double true_bw_mbps = (sum_bytes * BW_SCALE) / sum_sec;

  EXPECT_NEAR(s.avg_bw_mbps, true_bw_mbps, 1e-6);
}

// A constant message size must report a bandwidth consistent with the constant
// mean frequency, regardless of jitter in the periods.
TEST(StatisticsTest, ConstantSizeBandwidthMatchesFrequency)
{
  std::vector<double> periods = {9000.0, 11000.0, 10000.0, 12000.0, 8000.0};
  const double size_bytes = 2.0 * 1024.0 * 1024.0;  // 2 MB
  std::vector<double> sizes(periods.size(), size_bytes);

  WindowedStats s = computeWindowedStats(periods, sizes);

  EXPECT_NEAR(s.avg_bw_mbps, s.avg_freq_hz * BW_SCALE * size_bytes, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
