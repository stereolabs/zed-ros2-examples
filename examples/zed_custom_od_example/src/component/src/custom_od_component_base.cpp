// Copyright 2024 Stereolabs
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

#include "custom_od_component_base.hpp"
#include <sensor_msgs/image_encodings.hpp>

using namespace std::placeholders;

namespace stereolabs
{

ZedCustomOd::ZedCustomOd(const rclcpp::NodeOptions & options)
: Node("zed_custom_od_node", options), _defaultQoS(1)
{
  RCLCPP_INFO(get_logger(), "********************************************");
  RCLCPP_INFO(get_logger(), " ZED Custom Object Detection Base Component ");
  RCLCPP_INFO(get_logger(), "********************************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "********************************************");

  // ----> Create camera image subscriber
  _subImage = image_transport::create_camera_subscription(
    this, "zed_node/rgb/image_rect_color",
    std::bind(&ZedCustomOd::camera_callback, this, _1, _2), "raw",
    _defaultQoS.get_rmw_qos_profile());

  RCLCPP_INFO_STREAM(
    get_logger(),
    "Subscribed to topic: " << _subImage.getTopic());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Subscribed to topic: " << _subImage.getInfoTopic());
  // ----> Create camera image subscriber

}

void ZedCustomOd::camera_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & img,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info)
{
  // ----> Check for correct input image encoding
  if (img->encoding != sensor_msgs::image_encodings::BGRA8) {
    RCLCPP_ERROR(
      get_logger(),
      "The input topic image requires 'BGRA8' encoding");
    exit(EXIT_FAILURE);
  }
  // <---- Check for correct input image encoding

  RCLCPP_INFO(this->get_logger(), "Image Received");
}

}  // namespace stereolabs

// *************************************************************************
// Note: Only the child class must be registetered as a component plugin

// #include "rclcpp_components/register_node_macro.hpp"

// // Register the component with class_loader.
// // This acts as a sort of entry point, allowing the component to be
// // discoverable when its library is being loaded into a running process.
// RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedCustomOd)
// *************************************************************************
