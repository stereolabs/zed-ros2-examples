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

#ifndef CPU_METER_HPP_
#define CPU_METER_HPP_

#include <string>

namespace stereolabs
{

/// @brief Measures the CPU time consumed by the *whole current process*.
///
/// Important scope caveat: this is per-process, not per-node. When the
/// benchmark runs as a component inside a container it therefore also counts
/// every other component in that container. That is not a defect - comparing a
/// composed run against a separate-process run only makes sense on total CPU
/// across all the processes involved - but it does mean the figure must always
/// be reported together with what it covers. See sharesProcess().
class CpuMeter
{
public:
  /// @brief Take the initial CPU snapshot. Safe to call more than once.
  void start();

  /// @brief Whether the platform provided usable CPU accounting.
  bool valid() const {return mValid;}

  /// @brief CPU seconds consumed by this process since start().
  double cpuSeconds() const;

  /// @brief CPU usage since start(), as a percentage of one core.
  /// @param wall_sec Wall-clock seconds elapsed over the same interval.
  double cpuPercent(double wall_sec) const;

private:
  /// @brief Read this process' utime+stime, in seconds.
  static bool readProcessCpuSeconds(double & out_sec);

  double mStartCpuSec = 0.0;
  bool mValid = false;
};

}  // namespace stereolabs

#endif  // CPU_METER_HPP_
