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

#ifndef ZED_ARUCO_LOC_COMPONENT_HPP_
#define ZED_ARUCO_LOC_COMPONENT_HPP_

#include <opencv2/opencv.hpp>
#include <map>

#include <rcutils/logging_macros.h>

#include <image_transport/camera_publisher.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <atomic>

#include "aruco_loc_visibility_control.hpp"

namespace stereolabs
{

typedef struct
{
  int idx;
  cv::Vec3d position;
  cv::Vec3d orientation;
} ArucoPose;

class ZedArucoLocComponent : public rclcpp::Node
{
public:
  ZED_ARUCO_LOC_COMPONENT_PUBLIC
  explicit ZedArucoLocComponent(const rclcpp::NodeOptions & options);

  virtual ~ZedArucoLocComponent() {}

protected:
  void camera_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info);

private:
  // Publisher
  image_transport::CameraPublisher _pubDetect;  // Publisher for detection results

  // Subscriber
  image_transport::CameraSubscriber _subImage;  // ZED Image subscriber

  // QoS parameters
  rclcpp::QoS _defaultQoS;

  std::atomic<bool> _detRunning;

  // ----> Parameters
  size_t _tagCount = 1; // Number of tags available in the environment
  float _tagSize = 0.16; // Size of the tags [m]
  std::map<int, ArucoPose> _tagPoses; // Pose of each tag in the environment in World coordinates
  // <---- Parameters
};

}  // namespace stereolabs

#endif  // ZED_ARUCO_LOC_COMPONENT_HPP_
