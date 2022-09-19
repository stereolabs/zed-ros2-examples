/*
 * MIT License
 *
 * Copyright (c) 2022 Stereolabs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef TOPIC_BENCHMARK_HPP
#define TOPIC_BENCHMARK_HPP

#include <atomic>

#include "visibility_control.h"

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#define DEFAULT_TOPIC_NAME std::string("topic_name")

namespace stereolabs
{
class TopicBenchmarkComponent : public rclcpp::Node
{
public:
  TOPIC_BENCHMARK_PUBLIC
  TopicBenchmarkComponent(const rclcpp::NodeOptions& options);
  virtual ~TopicBenchmarkComponent();

protected:
  void init();

  // ----> Node Parameters
  template <typename T>
  void getParam(std::string paramName, T defValue, T& outVal, std::string log_info = std::string());

  void getParameters();
  // ----> Node Parameters

  void updateTopicInfo();  ///< Update the information to subscribe to the topic under benchmarking

private:
  double mSubFreqTot;  ///< Total of subscriber receiving frequency for average computation
  double mSubFreqBw;   ///< Average topic bandwidth (topic_size x avg_freq)

  std::vector<std::string> mTopicTypes;  ///< Types of the benchmarked topics (retrieved runtime)
  rclcpp::TimerBase::SharedPtr mTopicTimer;

  // Parameters
  std::string mTopicName = DEFAULT_TOPIC_NAME;  ///< Name of the benchmarked topic
  int mWinSize = 15;                            ///< Window size for frequency average

  // QoS parameters
  rclcpp::QoS mSubQos;  ///< Subscriber QoS profile

  std::atomic<bool> mTopicAvailable;  ///< Indicate if the benchmarked topic is published by other nodes
};
}  // namespace stereolabs

#endif