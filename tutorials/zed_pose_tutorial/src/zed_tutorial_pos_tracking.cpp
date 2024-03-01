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

/**
 * This tutorial demonstrates simple receipt of ZED pose and odometry messages over the ROS system.
 */

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::placeholders;

#define RAD2DEG 57.295779513

class MinimalPoseOdomSubscriber : public rclcpp::Node
{
public:
  MinimalPoseOdomSubscriber()
  : Node("zed_odom_pose_tutorial")
  {
    /* Note: it is very important to use a QOS profile for the subscriber that is compatible
         * with the QOS profile of the publisher.
         * The ZED component node uses a default QoS profile with reliability set as "RELIABLE"
         * and durability set as "VOLATILE".
         * To be able to receive the subscribed topic the subscriber must use compatible
         * parameters.
         */

    // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings

    rclcpp::QoS qos(10);

    auto sub_opt = rclcpp::SubscriptionOptions();

    // Create pose subscriber
    mPoseSub = create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose", qos,
      std::bind(&MinimalPoseOdomSubscriber::poseCallback, this, _1), sub_opt);

    // Create odom subscriber
    mOdomSub = create_subscription<nav_msgs::msg::Odometry>(
      "odom", qos,
      std::bind(&MinimalPoseOdomSubscriber::odomCallback, this, _1), sub_opt);
  }

protected:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Camera position in map frame
    double tx = msg->pose.position.x;
    double ty = msg->pose.position.y;
    double tz = msg->pose.position.z;

    // Orientation quaternion
    tf2::Quaternion q(
      msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
      msg->pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Output the measure
    RCLCPP_INFO(
      get_logger(),
      "Received pose in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f - "
      "Ts: %u.%u sec ",
      msg->header.frame_id.c_str(), tx, ty, tz, roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG,
      msg->header.stamp.sec, msg->header.stamp.nanosec);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Camera position in map frame
    double tx = msg->pose.pose.position.x;
    double ty = msg->pose.pose.position.y;
    double tz = msg->pose.pose.position.z;

    // Orientation quaternion
    tf2::Quaternion q(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Output the measure
    RCLCPP_INFO(
      get_logger(),
      "Received odometry in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f - "
      "Ts: %u.%u sec ",
      msg->header.frame_id.c_str(), tx, ty, tz, roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG,
      msg->header.stamp.sec, msg->header.stamp.nanosec);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mPoseSub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOdomSub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto pos_track_node = std::make_shared<MinimalPoseOdomSubscriber>();

  rclcpp::spin(pos_track_node);
  rclcpp::shutdown();
  return 0;
}
