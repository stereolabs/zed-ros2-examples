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

#ifndef CUSTOM_OD_COMPONENT_HPP_
#define CUSTOM_OD_COMPONENT_HPP_

#include "custom_od_visibility_control.hpp"
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>

namespace stereolabs
{

/*!
 * @brief Pure virtual class to be derived to create a Custom OD
 */
class ZedCustomOd : public rclcpp::Node
{
public:
  ZED_CUSTOM_OD_COMPONENT_PUBLIC
  explicit ZedCustomOd(const rclcpp::NodeOptions & options);

  virtual ~ZedCustomOd() {}

protected:
  /*!
   * @brief Initialize the custom inference engine
   *        This is a pure virtual function.
   *
   */
  virtual void init() = 0;

  /*!
   * @brief Perform the inference on the latest available image and
   *        publish the bounding boxes to the ZED Wrapper node
   *        This is a pure virtual function.
   *
   */
  virtual void doInference() = 0;

private:
  void camera_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info);

  void publishResult();

private:
  // ----> ROS Messages
  image_transport::CameraSubscriber _subImage;  // ZED Image subscriber
  rclcpp::QoS _defaultQoS;                      // QoS parameters
  // <---- ROS Messages

};

}  // namespace stereolabs

#endif
