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

#ifndef HARWARE_LOAD_BENCHMARK_COMPONENT_HPP_
#define HARWARE_LOAD_BENCHMARK_COMPONENT_HPP_
#include <rcutils/logging_macros.h>


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <string>
#include <fstream>
#include <chrono>
#include <numeric>
#include <vector>

#include "HardwareLoad.h"

namespace stereolabs
{
class HardwareLoadBenchmarkComponent : public rclcpp::Node
{
public:
  
  explicit HardwareLoadBenchmarkComponent(const rclcpp::NodeOptions & options);
  virtual ~HardwareLoadBenchmarkComponent();

protected:

  // ----> Node Parameters
  template<typename T>
  void getParam(
    std::string paramName, T defValue, T & outVal,
    std::string log_info = std::string(), bool dynamic = false);

  void getParameters();
  // ----> Node Parameters

  void getLoadMeasurements();
  // ----> CPU/GPU Load Measurements

  double getMean(const std::vector<float> & v);
  // ----> Get mean value from measurements vector

  void saveResults(); ///< Save Results to json file

private:
    
  // Params
  int mLoadPeriod;
  std::string mResultsFilePath = "results.txt";
  bool mUseRosLog = false;  ///< Use ROS logging system

  // Hardware measurement helper (from your header)
  HardwareLoad mHw;

  // Timer
  rclcpp::TimerBase::SharedPtr mTimer;

  // Measurements samples
  std::vector<float> mCpuSamples;
  std::vector<float> mGpuSamples;
  float mRamLoad;

};

}  // namespace stereolabs

#endif  // HARDWARE_LOAD_BENCHMARK_COMPONENT_HPP_