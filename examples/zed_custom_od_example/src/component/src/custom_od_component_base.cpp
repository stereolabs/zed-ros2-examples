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
#include <opencv2/opencv.hpp>

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

  _newImage = false;

  // Initialize times for stats
  _lastImgTime = get_clock()->now();
  _lastInferenceTime = get_clock()->now();

  // Read common parameters
  readCommonParams();

  // ----> Create camera image subscriber
  std::string topic_prefix = _mainNodeName + "/";
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

  // ----> Start the processing loop
  int64_t proc_period_msec = static_cast<int64_t>(std::round(1000.f / _loopFreq));
  _elabTimer = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::milliseconds(proc_period_msec)),
    std::bind(&ZedCustomOd::processing_callback, this));
  // <---- Start the processing loop
}

void ZedCustomOd::readCommonParams()
{
  RCLCPP_INFO(get_logger(), "*** Common Parameters ***");

  getParam("general.main_node_name", _mainNodeName, _mainNodeName, " * Main node name:\t\t", false);
  getParam(
    "general.detection_topic", _pubTopicName, _pubTopicName, " * Detection topic:\t\t",
    false);
  getParam("general.loop_frequency", _loopFreq, _loopFreq, " * Loop frequency [Hz]:\t", false);

  getParam("general.debug_mode", _debugMode, _debugMode);
  RCLCPP_INFO_STREAM(get_logger(), " * Debug mode: " << (_debugMode ? "TRUE" : "FALSE"));

  if (_debugMode) {
    rcutils_ret_t res = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (res != RCUTILS_RET_OK) {
      RCLCPP_WARN(get_logger(), "Error setting DEBUG level for logger");
    } else {
      RCLCPP_INFO(get_logger(), " + Debug Mode enabled +");
    }
  } else {
    rcutils_ret_t res = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

    if (res != RCUTILS_RET_OK) {
      RCLCPP_WARN(get_logger(), "Error setting INFO level for logger");
    }
  }
}

void ZedCustomOd::camera_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & img,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info)
{
  // ----> Stats
  auto imgTime = get_clock()->now();
  double img_elapsed_sec = (imgTime - _lastImgTime).nanoseconds() / 1e9;
  _lastImgTime = imgTime;
  double img_freq = 1.0 / img_elapsed_sec;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Image topic freq: " << img_freq << " Hz");
  // <---- Stats

  if (_inferenceRunning) { // No new images while processing the latest received
    return;
  }

  if (_newImage) { // While the latest image has not been processed, we ignore the oncoming
    return;
  }

  std::lock_guard<std::mutex> lock(_detMux);

  // ----> Check for correct input image encoding
  if (img->encoding != sensor_msgs::image_encodings::BGRA8) {
    RCLCPP_ERROR(
      get_logger(),
      "The input topic image requires 'BGRA8' encoding");
    exit(EXIT_FAILURE);
  }
  // <---- Check for correct input image encoding

  RCLCPP_DEBUG(this->get_logger(), "Image Received");

  // Set the frame id of the detection to be the same as the image frame id
  _detFrameId = img->header.frame_id;

  // ----> Convert BGRA image for processing by using OpenCV
  void * data =
    const_cast<void *>(reinterpret_cast<const void *>(&img->data[0]));
  cv::Mat bgra(img->height, img->width, CV_8UC4, data);

  cv::cvtColor(bgra, _zedImg, cv::COLOR_BGRA2BGR);
  // ----> Convert BGRA image for processing by using OpenCV

  // At the end of the function
  _newImage = true;
}

void ZedCustomOd::publishResult()
{
  std::unique_ptr<vision_msgs::msg::Detection2DArray> msg =
    std::make_unique<vision_msgs::msg::Detection2DArray>();

  msg->header.frame_id = _detFrameId;
  msg->header.stamp = get_clock()->now(); // TODO(Walt) Check if it's better to set the timestamp just after the inference returned
  msg->detections = _detections;

  _pubDet2dArray->publish(std::move(msg));
}

void ZedCustomOd::processing_callback()
{
  if (!_newImage) {
    return;
  }

  std::lock_guard<std::mutex> lock(_detMux);

  // ----> Stats
  auto procTime = get_clock()->now();
  double proc_elapsed_sec = (procTime - _lastInferenceTime).nanoseconds() / 1e9;
  _lastInferenceTime = procTime;
  double proc_freq = 1.0 / proc_elapsed_sec;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Inference freq: " << proc_freq << " Hz");
  // <---- Stats

  // Call the custom inference
  _inferenceRunning = true;
  doInference();
  _inferenceRunning = false;
  _newImage = false;
  double inference_duration_sec = (get_clock()->now() - procTime).nanoseconds() / 1e9;
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Inference duration: " << inference_duration_sec << " sec");

  // Publish the detection results
  publishResult();
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
