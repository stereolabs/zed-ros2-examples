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

#include "aruco.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace stereolabs
{

ZedArucoLocComponent::ZedArucoLocComponent(const rclcpp::NodeOptions & options)
: Node("zed_aruco_loc_node", options), _defaultQoS(1), _detRunning(false)
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

  if (_detRunning) {
    return;
  }

  _detRunning = true;

  // Time statistics
  rclcpp::Time start;
  double elapsed_sec;


  // Is result image subscribed?
  bool res_sub = (_pubDetect.getNumSubscribers() > 0);

  // ----> Check for correct input image encoding
  if (img->encoding != sensor_msgs::image_encodings::BGRA8) {
    RCLCPP_ERROR(get_logger(), "The input topic image requires 'BGRA8' encoding");
    return;
  }
  // <---- Check for correct input image encoding

  // ----> Convert BGRA image for processing by using OpenCV
  start = get_clock()->now();
  void * data = const_cast<void *>(reinterpret_cast<const void *>(&img->data[0]));
  cv::Mat bgra(img->height, img->width, CV_8UC4, data);
  cv::Mat bgr, gray; // bgr is used to publish the detection image, gray for ArUco processing

  cv::cvtColor(bgra, gray, cv::COLOR_BGRA2GRAY);
  if (res_sub) {
    cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
  }
  elapsed_sec = (get_clock()->now() - start).nanoseconds() / 1e9;
  RCLCPP_INFO_STREAM(get_logger(), " * Color conversion: " << elapsed_sec << " sec");
  // ----> Convert BGRA image for processing by using OpenCV

  // ----> Detect ArUco Markers
  start = get_clock()->now();
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;

  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
  cv::aruco::detectMarkers(bgr, dictionary, corners, ids);
  elapsed_sec = (get_clock()->now() - start).nanoseconds() / 1e9;
  RCLCPP_INFO_STREAM(get_logger(), " * Marker detection: " << elapsed_sec << " sec");
  // <---- Detect ArUco Markers

  if (corners.empty()) {
    return;
  }

  RCLCPP_INFO_STREAM(get_logger(), " * Detected tags: " << ids.size());

  // ----> Refine the result
  start = get_clock()->now();
  for (size_t i = 0; i < corners.size(); ++i) {
    cv::cornerSubPix(
      gray, corners[i], cv::Size(5, 5), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
  }
  elapsed_sec = (get_clock()->now() - start).nanoseconds() / 1e9;
  if (corners.empty()) {
    return;
  }
  RCLCPP_INFO_STREAM(get_logger(), " * Subpixel refinement: " << elapsed_sec << " sec");
  // <---- Refine the result

  // ----> Estimate Marker positions
  start = get_clock()->now();
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(corners, _tagSize, cam_info->k, cam_info->d, rvecs, tvecs);
  RCLCPP_INFO_STREAM(get_logger(), " * Marker poses estimation: " << elapsed_sec << " sec");
  // <---- Estimate Marker positions

  // ----> Draw and publish the results
  if (res_sub) {
    cv::aruco::drawDetectedMarkers(bgr, corners, ids);

    // Create the output message and copy coverted data
    std::shared_ptr<sensor_msgs::msg::Image> out_bgr = std::make_shared<sensor_msgs::msg::Image>();

    out_bgr->header.stamp = img->header.stamp;
    out_bgr->header.frame_id = img->header.frame_id;
    out_bgr->height = bgr.rows;
    out_bgr->width = bgr.cols;

    int num = 1; // for endianness detection
    out_bgr->is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

    out_bgr->step = bgr.step;

    size_t size = out_bgr->step * out_bgr->height;
    out_bgr->data.resize(size);

    out_bgr->encoding = sensor_msgs::image_encodings::BGR8;
    memcpy(reinterpret_cast<char *>((&out_bgr->data[0])), &bgr.data[0], size);

    // Publish the new image message coupled with camera info from the original message
    _pubDetect.publish(out_bgr, cam_info);
  }
  // <---- Draw and publish the results

  // TODO 2. Coordinates transformation
  // TODO 3. `set_pose` service call

  _detRunning = false;
}

}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedArucoLocComponent)
