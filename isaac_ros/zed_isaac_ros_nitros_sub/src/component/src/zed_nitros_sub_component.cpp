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

#include "zed_nitros_sub_component.hpp"

namespace stereolabs
{

ZedNitrosSubComponent::ZedNitrosSubComponent(const rclcpp::NodeOptions & options)
: Node("zed_nitros_sub_component", options),
  mDefaultQoS(10)
{
}

void ZedNitrosSubComponent::sub_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & img)
{
  RCLCPP_INFO(
    this->get_logger(), "Received image with width: %d, height: %d", img->width, img->height);
}

}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedNitrosSubComponent)
