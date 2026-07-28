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

#include "cpu_meter.hpp"

#include <unistd.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace stereolabs
{

bool CpuMeter::readProcessCpuSeconds(double & out_sec)
{
  std::ifstream stat("/proc/self/stat");
  if (!stat.is_open()) {
    return false;
  }

  std::string line;
  std::getline(stat, line);
  if (line.empty()) {
    return false;
  }

  // The second field (comm) is parenthesized and may itself contain spaces and
  // parentheses, so tokenizing from the start is unsafe: skip past the LAST
  // ')' and count fields from there. utime/stime are fields 14/15 overall,
  // i.e. the 12th and 13th token after that point.
  const size_t close = line.rfind(')');
  if (close == std::string::npos) {
    return false;
  }

  std::istringstream rest(line.substr(close + 1));
  std::vector<std::string> tokens;
  std::string token;
  while (rest >> token) {
    tokens.push_back(token);
    if (tokens.size() >= 13) {
      break;
    }
  }
  if (tokens.size() < 13) {
    return false;
  }

  const int64_t ticks_per_sec = ::sysconf(_SC_CLK_TCK);
  if (ticks_per_sec <= 0) {
    return false;
  }

  try {
    // tokens[0] is field 3 (state), so utime (14) is tokens[11] and
    // stime (15) is tokens[12].
    const double utime = std::stod(tokens[11]);
    const double stime = std::stod(tokens[12]);
    out_sec = (utime + stime) / static_cast<double>(ticks_per_sec);
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

void CpuMeter::start()
{
  mValid = readProcessCpuSeconds(mStartCpuSec);
}

double CpuMeter::cpuSeconds() const
{
  if (!mValid) {
    return 0.0;
  }
  double now_sec = 0.0;
  if (!readProcessCpuSeconds(now_sec)) {
    return 0.0;
  }
  const double delta = now_sec - mStartCpuSec;
  return (delta > 0.0) ? delta : 0.0;
}

double CpuMeter::cpuPercent(double wall_sec) const
{
  if (!mValid || wall_sec <= 0.0) {
    return 0.0;
  }
  return 100.0 * cpuSeconds() / wall_sec;
}

}  // namespace stereolabs
