// Copyright 2023 Stereolabs
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

#include "zed_aruco_localization_component.hpp"

#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace stereolabs
{

ZedArucoLocComponent::ZedArucoLocComponent(const rclcpp::NodeOptions & options)
: Node("zed_aruco_loc_node", options), _defaultQoS(1)
{
  RCLCPP_INFO(get_logger(), "*********************************");
  RCLCPP_INFO(get_logger(), " ZED ArUco Localization Component ");
  RCLCPP_INFO(get_logger(), "*********************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "*********************************");

  /* Note: it is very important to use a QOS profile for the subscriber that is compatible
     * with the QOS profile of the publisher.
     * The ZED component node uses a default QoS profile with reliability set as "RELIABLE"
     * and durability set as "VOLATILE".
     * To be able to receive the subscribed topic the subscriber must use compatible
     * parameters.
     */

  // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings

  _defaultQoS.keep_last(10);
  _defaultQoS.reliable();
  _defaultQoS.durability_volatile();

  // Create image publisher
  _pubDetect = image_transport::create_camera_publisher(
    this, "aruco_result",
    _defaultQoS.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << _pubDetect.getTopic());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << _pubDetect.getInfoTopic());

  // Create camera image subscriber
  _subImage = image_transport::create_camera_subscription(
    this, "zed_image", std::bind(&ZedArucoLocComponent::camera_callback, this, _1, _2), "raw",
    _defaultQoS.get_rmw_qos_profile());

  RCLCPP_INFO_STREAM(get_logger(), "Subscribed on topic: " << _subImage.getTopic());
  RCLCPP_INFO_STREAM(get_logger(), "Subscribed on topic: " << _subImage.getInfoTopic());
}

void ZedArucoLocComponent::camera_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & img,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info)
{
  // Check for correct input image encoding
  if (img->encoding != sensor_msgs::image_encodings::BGRA8) {
    RCLCPP_ERROR(get_logger(), "The input topic image requires 'BGRA8' encoding");
    exit(EXIT_FAILURE);
  }

  // TODO 1. ArUco detection
  // TODO 2. Coordinates transformation
  // TODO 3. `set_pose` service call
}

}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedArucoLocComponent)
