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

#ifndef ZED_NITROS_SUB__COMPONENT_HPP_
#define ZED_NITROS_SUB__COMPONENT_HPP_

#include <rcutils/logging_macros.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "nitros_sub_visibility_control.hpp"

namespace stereolabs
{

class ZedNitrosSubComponent : public rclcpp::Node
{
public:
  ZED_NITROS_SUB_COMPONENT_PUBLIC
  explicit ZedNitrosSubComponent(const rclcpp::NodeOptions & options);

  virtual ~ZedNitrosSubComponent() {}

protected:
  void sub_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & img);

private:
  // QoS parameters
  rclcpp::QoS mDefaultQoS;
};

}  // namespace stereolabs

#endif  // ZED_NITROS_SUB__COMPONENT_HPP_
