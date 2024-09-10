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
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <mutex>

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
   * @brief Initialize the custom inference engine.
   *        This function is automatically called by
   *        the constructor, so it's not required to call it.
   *        This is a pure virtual function.   *
   */
  virtual void init() = 0;

  /*!
   * @brief Perform the inference on the latest available image and
   *        publish the bounding boxes to the ZED Wrapper node.
   *        This function is automatically executed in the node loop,
   *        so it's not required to call it.
   *        This is a pure virtual function.
   */
  virtual void doInference() = 0;

  template<typename T>
  void getParam(
    std::string paramName, T defValue, T & outVal,
    std::string log_info = std::string(), bool dynamic = false);

private:
  void processing_callback();
  void camera_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info);

  void publishResult();

  void readCommonParams();

protected:
  cv::Mat _zedImg; //!< This is the RGB image to be used as input for the inference

private:
  // ----> Common Node Parameters
  std::string _mainNodeName = "zed_node";
  std::string _pubTopicName = "detections";
  float _loopFreq = 150.0f;
  // <---- Common Node Parameters

  // ----> ROS Messages
  std::string _subTopicName = "rgb/image_rect_color";
  image_transport::CameraSubscriber _subImage;  // ZED Image subscriber
  rclcpp::QoS _defaultQoS;                      // QoS parameters

  std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::Detection2DArray>> _pubDet2dArray;
  // <---- ROS Messages

  std::mutex _detMux;

  std::vector<vision_msgs::msg::Detection2D> _detections;
  std::string _detFrameId;
  rclcpp::TimerBase::SharedPtr _elabTimer;

  // ----> Running variables
  rclcpp::Time _lastImgTime;  // Time of the latest received image
  rclcpp::Time _lastInferenceTime;  // Time of the latest processed image
  std::atomic<bool> _newImage; // Indicates that a new image is available
  std::atomic<bool> _inferenceRunning; // Indicates that an inference processing is running
  // <---- Running variables
};

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

}  // namespace stereolabs

#endif
