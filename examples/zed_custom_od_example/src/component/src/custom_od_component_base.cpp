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

const int QOS_QUEUE_SIZE = 100;

ZedCustomOd::ZedCustomOd(const rclcpp::NodeOptions & options)
: Node("zed_custom_od_node", options), _defaultQoS(1)
{
  RCLCPP_INFO(get_logger(), "********************************************");
  RCLCPP_INFO(get_logger(), " ZED Custom Object Detection Base Component ");
  RCLCPP_INFO(get_logger(), "********************************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "********************************************");

  readCommonParams();

  std::string topic_prefix = _mainNodeName + "/";
  // ----> Create camera image subscriber
  RCLCPP_INFO(get_logger(), "*** Common Subscribers ***");

  std::string sub_topic = topic_prefix + _subTopicName;
  _subImage = image_transport::create_camera_subscription(
    this, sub_topic, std::bind(&ZedCustomOd::camera_callback, this, _1, _2), "raw",
    _defaultQoS.get_rmw_qos_profile());

  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Subscribed to topic: " << _subImage.getTopic());
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Subscribed to topic: " << _subImage.getInfoTopic());
  // ----> Create camera image subscriber

  // ----> Create detection publisher
  RCLCPP_INFO(get_logger(), "*** Common Publishers ***");

  auto pub_opt = rclcpp::PublisherOptions();
  pub_opt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();

  std::string pub_topic_name = "~/" + _pubTopicName;
  _pubDet2dArray = create_publisher<
    vision_msgs::msg::Detection2DArray>(
    pub_topic_name, rclcpp::QoS(QOS_QUEUE_SIZE), pub_opt);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Advertised on topic: " << _pubDet2dArray->get_topic_name());
  // <---- Create detection publisher
}

template<typename T>
void ZedCustomOd::getParam(
  std::string paramName, T defValue, T & outVal,
  std::string log_info, bool dynamic)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = !dynamic;

  declare_parameter(paramName, rclcpp::ParameterValue(defValue), descriptor);

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

void ZedCustomOd::readCommonParams()
{
  RCLCPP_INFO(get_logger(), "*** Common Parameters ***");

  getParam("general.main_node_name", _mainNodeName, _mainNodeName, " * Main node name:\t", false);
  getParam("general.detection_topic", _pubTopicName, _pubTopicName, " * Detection topic:\t", false);
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

  _detFrameId = img->header.frame_id;
}

void ZedCustomOd::publishResult()
{
  std::lock_guard<std::mutex> lock(_detMux);

  std::unique_ptr<vision_msgs::msg::Detection2DArray> msg =
    std::make_unique<vision_msgs::msg::Detection2DArray>();

  msg->header.frame_id = _detFrameId;
  msg->header.stamp = get_clock()->now();
  msg->detections = _detections;

  _pubDet2dArray->publish(std::move(msg));
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
