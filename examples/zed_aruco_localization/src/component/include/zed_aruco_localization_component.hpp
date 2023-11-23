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

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#include <image_transport/camera_publisher.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <atomic>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <zed_interfaces/srv/set_pose.hpp>

#include "aruco_loc_visibility_control.hpp"

namespace stereolabs
{

typedef struct
{
  int idx;
  std::string marker_frame_id;
  std::vector<double> position;
  std::vector<double> orientation;
} ArucoPose;

class ZedArucoLoc : public rclcpp::Node
{
public:
  ZED_ARUCO_LOC_COMPONENT_PUBLIC
  explicit ZedArucoLoc(const rclcpp::NodeOptions & options);

  virtual ~ZedArucoLoc() {}

protected:
  void camera_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info);

  template<typename T> void getParam(
    std::string paramName, T defValue, T & outVal,
    std::string log_info = std::string(), bool dynamic = false);

  void getParams();
  void getGeneralParams();
  void getMarkerParams();

  void initTFs();
  void broadcastMarkerTFs();
  bool getTransformFromTf(
    std::string targetFrame, std::string sourceFrame,
    tf2::Transform & out_tr);

  bool resetZedPose(tf2::Transform & new_pose);

private:
  // ----> ROS Messages
  image_transport::CameraPublisher _pubDetect;  // Publisher for detection results
  image_transport::CameraSubscriber _subImage;  // ZED Image subscriber
  rclcpp::QoS _defaultQoS;// QoS parameters
  // <---- ROS Messages

  // Service client
  rclcpp::Client<zed_interfaces::srv::SetPose>::SharedPtr _setPoseClient;

  // ----> Running variables
  rclcpp::Time _detTime; // Time of the latest detection
  std::atomic<bool> _detRunning; // Flag used to not perform cuncurrent detections
  // <---- Running variables

  // ----> Parameters
  int _markerCount = 1;    // Number of markers available in the environment
  float _markerSize = 0.16f;  // Size of the tags [m]
  float _detRate = 1.0f;      // Maximum detection frequency for pose update
  std::string _worldFrameId;  // World frame id
  std::string _cameraName = "zed"; // Name of the camera to be re-localized
  double _maxDist;            // Maximum distance from the camera
  bool _refineDetection; // Enable sub-pixel refinement for the detected corners
  std::map<int, ArucoPose> _tagPoses; // Pose of each tag in the environment in World coordinates
  bool _debugActive;          // Enable debug messages
  // <---- Parameters

  // ----> TF2
  std::unique_ptr<tf2_ros::Buffer> _tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> _tfListener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> _tfBroadcaster;

  tf2::Transform _img2aruco;
  tf2::Transform _aruco2img;
  tf2::Transform _ros2img;
  tf2::Transform _img2ros;
  tf2::Transform _left2base;

  rclcpp::TimerBase::SharedPtr _tfTimer; // Timer to broadcast marker TFs
  // <---- TF2
};

}  // namespace stereolabs

#endif  // ZED_ARUCO_LOC_COMPONENT_HPP_
