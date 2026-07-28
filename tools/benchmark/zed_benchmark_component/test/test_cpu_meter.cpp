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

#include <chrono>

#include "cpu_meter.hpp"

using stereolabs::CpuMeter;

TEST(CpuMeterTest, ReportsCpuTimeOnThisPlatform) {
  CpuMeter meter;
  meter.start();
  ASSERT_TRUE(meter.valid()) << "/proc/self/stat parsing failed";
  EXPECT_GE(meter.cpuSeconds(), 0.0);
}

TEST(CpuMeterTest, CpuTimeGrowsWithBusyWork) {
  CpuMeter meter;
  meter.start();
  ASSERT_TRUE(meter.valid());

  // Burn measurable CPU. The loop must be observable by the kernel's tick
  // accounting, so keep it well above one scheduler tick.
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
  volatile double sink = 0.0;
  while (std::chrono::steady_clock::now() < deadline) {
    for (int i = 1; i < 20000; ++i) {
      sink += 1.0 / static_cast<double>(i);
    }
  }
  (void)sink;

  EXPECT_GT(meter.cpuSeconds(), 0.0);
  // Busy-looping on one thread cannot exceed one core, with generous slack for
  // tick granularity.
  EXPECT_LT(meter.cpuPercent(0.3), 200.0);
}

TEST(CpuMeterTest, PercentIsZeroForNonPositiveWallTime) {
  CpuMeter meter;
  meter.start();
  EXPECT_DOUBLE_EQ(meter.cpuPercent(0.0), 0.0);
  EXPECT_DOUBLE_EQ(meter.cpuPercent(-1.0), 0.0);
}

TEST(CpuMeterTest, UnstartedMeterReportsNothing) {
  CpuMeter meter;
  EXPECT_FALSE(meter.valid());
  EXPECT_DOUBLE_EQ(meter.cpuSeconds(), 0.0);
  EXPECT_DOUBLE_EQ(meter.cpuPercent(1.0), 0.0);
}
