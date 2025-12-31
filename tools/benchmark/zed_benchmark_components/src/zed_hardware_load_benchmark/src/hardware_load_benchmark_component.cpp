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

#include "hardware_load_benchmark_component.hpp"
#include <rcutils/logging_macros.h>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <fstream>
#include <numeric>
#include <string>
#include <vector>

#include "HardwareLoad.h"

namespace stereolabs
{  
  HardwareLoadBenchmarkComponent::HardwareLoadBenchmarkComponent(const rclcpp::NodeOptions & options)
  : rclcpp::Node("hardware_load_benchmark", options)
  {
    // Initialize HardwareLoad internals (createCPUusage/createGPUusage/... are called inside launch()).
    // We immediately join to stop its internal 10ms sampling thread, because THIS node uses its own timer.
    mHw.launch();
    mHw.join();

    // Retrieve parameters
    getParameters();

    // Create timer for sampling
    auto period = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::milliseconds(mLoadPeriod));
    mTimer = this->create_wall_timer(period, std::bind(&HardwareLoadBenchmarkComponent::getLoadMeasurements, this));

    RCLCPP_INFO(this->get_logger(),
              "HardwareLoadComponent started. Sampling every %d ms, output: %s",
              mLoadPeriod, mResultsFilePath.c_str());

    // Ensure destructor is called on shutdown
    rclcpp::on_shutdown([this]() {
      this->saveResults();
    });

  }

  HardwareLoadBenchmarkComponent::~HardwareLoadBenchmarkComponent()
  {
    // In case destructor runs without on_shutdown (e.g., component unload).
    saveResults();
  }

    template<typename T>
  void HardwareLoadBenchmarkComponent::getParam(
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

  void HardwareLoadBenchmarkComponent::getParameters()
  {
    RCLCPP_INFO(get_logger(), "***** Load Benchmark parameters *****");
    getParam("cpu_gpu_load_period" , mLoadPeriod, mLoadPeriod, "CPU/GPU Load Period: ");
    getParam("results_file_path", mResultsFilePath, mResultsFilePath, "Results File path: "); 
    getParam("use_ros_log", mUseRosLog, mUseRosLog, "ROS Log: ");
  }


  void HardwareLoadBenchmarkComponent::getLoadMeasurements()
  {
    mHw.compute();

    const float cpu = mHw.getCPUload();
    const float gpu = mHw.getGPUload();
    mRamLoad = mHw.getRAMload();

    if (cpu >= 0.0f && cpu <100.0f) {
      mCpuSamples.push_back(cpu);
    }
    
    if (gpu >= 0.0f && gpu <100.0f) {
      mGpuSamples.push_back(gpu);
    }

    std::stringstream ss;
    if (!mUseRosLog) {
      ss << '\r';
    }
    ss << std::fixed << std::setprecision(2) <<" - CPU Load: " << cpu << "% - GPU Load: " << gpu
     << " % - RAM Load: " << mRamLoad;

    if (!mUseRosLog) {
      ss << std::flush;
      std::cout << ss.str();
    } else {
    RCLCPP_INFO_STREAM(get_logger(), ss.str());
    }
  }

  double HardwareLoadBenchmarkComponent::getMean(const std::vector<float> & v)
  {
    if (v.empty()) return 0.0;
    const double sum = std::accumulate(v.begin(), v.end(), 0.0);
    return sum / static_cast<double>(v.size());
  }

  void HardwareLoadBenchmarkComponent::saveResults()
  {
   
    // Cancel timer first
    if (mTimer) {
      mTimer->cancel();
    }

    RCLCPP_INFO_STREAM(
    get_logger(),
    "Appending results to file path: " << mResultsFilePath);

    std::ofstream output_file(mResultsFilePath,std::ios::out | std::ios::app);  

    if (!output_file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open results file");
      return;
    }

    output_file
      << "Average CPU Load: " << getMean(mCpuSamples) << "\n"
      << "Average GPU Load: " << getMean(mGpuSamples) << "\n"
      << "RAM Load: " << mRamLoad << "\n"

      << "-----------------------------\n";

  }

}// namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::HardwareLoadBenchmarkComponent)
