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

#include "winavg.hpp"

using stereolabs::WinAvg;

// An empty window must not divide by zero: the average is defined as 0.
TEST(WinAvgTest, EmptyAverageIsZero)
{
  WinAvg avg(5);
  EXPECT_EQ(avg.size(), 0u);
  EXPECT_DOUBLE_EQ(avg.getAvg(), 0.0);
}

// A single value is its own average.
TEST(WinAvgTest, SingleValue)
{
  WinAvg avg(5);
  EXPECT_DOUBLE_EQ(avg.addValue(10.0), 10.0);
  EXPECT_EQ(avg.size(), 1u);
  EXPECT_DOUBLE_EQ(avg.getAvg(), 10.0);
}

// Below the window size the average is the plain mean of all inserted values.
TEST(WinAvgTest, MeanBelowWindow)
{
  WinAvg avg(10);
  avg.addValue(2.0);
  avg.addValue(4.0);
  double last = avg.addValue(6.0);
  EXPECT_EQ(avg.size(), 3u);
  EXPECT_DOUBLE_EQ(last, 4.0);
  EXPECT_DOUBLE_EQ(avg.getAvg(), 4.0);
}

// Once full, the oldest value is evicted (FIFO) and the window size is kept.
TEST(WinAvgTest, WindowEviction)
{
  WinAvg avg(3);
  avg.addValue(1.0);
  avg.addValue(2.0);
  avg.addValue(3.0);
  // Window is full with {1,2,3}; adding 4 must drop the oldest (1).
  double res = avg.addValue(4.0);
  EXPECT_EQ(avg.size(), 3u);
  EXPECT_DOUBLE_EQ(res, (2.0 + 3.0 + 4.0) / 3.0);
}

// The running sum must stay consistent over many evictions (no drift).
TEST(WinAvgTest, StableOverManyInsertions)
{
  WinAvg avg(4);
  // Keep adding the same value: the average must remain exactly that value.
  for (int i = 0; i < 1000; ++i) {
    avg.addValue(7.5);
  }
  EXPECT_EQ(avg.size(), 4u);
  EXPECT_NEAR(avg.getAvg(), 7.5, 1e-9);
}

// Shrinking the window keeps the most recent values and drops the oldest.
TEST(WinAvgTest, SetNewSizeShrinks)
{
  WinAvg avg(5);
  for (double v : {1.0, 2.0, 3.0, 4.0, 5.0}) {
    avg.addValue(v);
  }
  ASSERT_EQ(avg.size(), 5u);

  // Keep only the 3 most recent values {3,4,5}.
  double res = avg.setNewSize(3);
  EXPECT_EQ(avg.size(), 3u);
  EXPECT_DOUBLE_EQ(res, (3.0 + 4.0 + 5.0) / 3.0);
  EXPECT_DOUBLE_EQ(avg.getAvg(), 4.0);
}

// Growing the window keeps every stored value and lets new ones accumulate.
TEST(WinAvgTest, SetNewSizeGrows)
{
  WinAvg avg(2);
  avg.addValue(10.0);
  avg.addValue(20.0);
  EXPECT_EQ(avg.size(), 2u);

  avg.setNewSize(4);
  EXPECT_EQ(avg.size(), 2u);  // growing does not invent values
  avg.addValue(30.0);
  avg.addValue(40.0);
  EXPECT_EQ(avg.size(), 4u);
  EXPECT_DOUBLE_EQ(avg.getAvg(), (10.0 + 20.0 + 30.0 + 40.0) / 4.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
