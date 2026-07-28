// Copyright 2025 Stereolabs
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

#include "typed_subscriber.hpp"

#include <map>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <utility>

#ifdef ZED_BENCHMARK_HAS_SL_ADAPTER
#include <zed_components/sl_type_adapter.hpp>
#endif

namespace stereolabs
{

namespace
{

// ----> Message content size, per supported type
// These deliberately count the message *content* and not the serialized form:
// a typed subscription never serializes, so asking for a wire-accurate size
// here would mean paying exactly the cost the intra-process path avoids.
size_t sizeBytes(const sensor_msgs::msg::Image & msg)
{
  return msg.data.size();
}

size_t sizeBytes(const sensor_msgs::msg::CompressedImage & msg)
{
  return msg.data.size();
}

size_t sizeBytes(const sensor_msgs::msg::PointCloud2 & msg)
{
  return msg.data.size();
}

size_t sizeBytes(const sensor_msgs::msg::CameraInfo & msg)
{
  // Fixed-size intrinsic/rectification/projection matrices plus the
  // variable-length distortion coefficients.
  return sizeof(msg.k) + sizeof(msg.r) + sizeof(msg.p) +
         msg.d.size() * sizeof(double) +
         sizeof(msg.height) + sizeof(msg.width) +
         sizeof(msg.binning_x) + sizeof(msg.binning_y);
}

size_t sizeBytes(const sensor_msgs::msg::Imu & msg)
{
  return sizeof(msg.orientation) + sizeof(msg.orientation_covariance) +
         sizeof(msg.angular_velocity) + sizeof(msg.angular_velocity_covariance) +
         sizeof(msg.linear_acceleration) +
         sizeof(msg.linear_acceleration_covariance);
}
// <---- Message content size, per supported type

/// @brief Build a Sample from a message carrying a std_msgs/Header.
template<typename MsgT>
Sample makeSample(const MsgT & msg)
{
  Sample sample;
  sample.size_bytes = sizeBytes(msg);
  const rclcpp::Time stamp(msg.header.stamp);
  // A zero stamp means the publisher did not fill it in: reporting a latency
  // measured against the epoch would be worse than reporting none.
  sample.has_stamp = stamp.nanoseconds() != 0;
  sample.stamp = stamp;
  return sample;
}

/// @brief Create a plain typed subscription for `MsgT`.
///
/// This is intra-process *capable*: with intra-process comms enabled it skips
/// the middleware entirely. It is not zero-copy, because rclcpp copies the
/// message into the subscription's buffer (and, when the publisher is
/// type-adapted, converts it first).
template<typename MsgT>
TypedSubscription makeTyped(
  rclcpp::Node & node, const std::string & topic, const std::string & type_name,
  const rclcpp::QoS & qos, const rclcpp::SubscriptionOptions & options,
  const SampleSink & sink)
{
  TypedSubscription result;
  result.sub = node.create_subscription<MsgT>(
    topic, qos,
    [sink](std::shared_ptr<const MsgT> msg) {sink(makeSample(*msg));},
    options);
  result.zero_copy = false;
  result.description = "typed (" + type_name + ")";
  return result;
}

using Factory = std::function<TypedSubscription(
      rclcpp::Node &, const std::string &, const std::string &,
      const rclcpp::QoS &, const rclcpp::SubscriptionOptions &,
      const SampleSink &)>;

/// @brief Registry of every message type a typed subscription exists for.
const std::map<std::string, Factory> & factories()
{
  static const std::map<std::string, Factory> kFactories = {
    {"sensor_msgs/msg/Image", makeTyped<sensor_msgs::msg::Image>},
    {"sensor_msgs/msg/CompressedImage",
      makeTyped<sensor_msgs::msg::CompressedImage>},
    {"sensor_msgs/msg/PointCloud2", makeTyped<sensor_msgs::msg::PointCloud2>},
    {"sensor_msgs/msg/CameraInfo", makeTyped<sensor_msgs::msg::CameraInfo>},
    {"sensor_msgs/msg/Imu", makeTyped<sensor_msgs::msg::Imu>},
  };
  return kFactories;
}

#ifdef ZED_BENCHMARK_HAS_SL_ADAPTER
/// The ZED wrapper publishes images through this adapter, so subscribing with
/// the very same adapter is what makes the publisher hand over its `sl::Mat`
/// by pointer instead of a converted copy.
using SlImageAdapter =
  rclcpp::TypeAdapter<stereolabs::StampedSlMat, sensor_msgs::msg::Image>;

/// @brief Create the ZED type-adapted, genuinely zero-copy subscription.
TypedSubscription makeAdapted(
  rclcpp::Node & node, const std::string & topic, const rclcpp::QoS & qos,
  const rclcpp::SubscriptionOptions & options, const SampleSink & sink)
{
  TypedSubscription result;
  result.sub = node.create_subscription<SlImageAdapter>(
    topic, qos,
    [sink](std::shared_ptr<const stereolabs::StampedSlMat> msg) {
      Sample sample;
      // Same byte count the adapter would put into Image.data, obtained
      // without touching the pixels.
      auto & mat = const_cast<sl::Mat &>(msg->mat);
      sample.size_bytes = mat.getStepBytes() * mat.getHeight();
      sample.has_stamp = msg->stamp.nanoseconds() != 0;
      sample.stamp = msg->stamp;
      // Receiving StampedSlMat is proof of intra-process delivery: the type has
      // no wire representation, so the middleware could not have produced it.
      sample.intra_process_confirmed = true;
      sink(sample);
    },
    options);
  result.zero_copy = true;
  result.description = "type-adapted (zero-copy, sl::Mat by pointer)";
  return result;
}
#endif  // ZED_BENCHMARK_HAS_SL_ADAPTER

}  // namespace

std::vector<std::string> supportedTypedMessageTypes()
{
  std::vector<std::string> types;
  types.reserve(factories().size());
  for (const auto & entry : factories()) {
    types.push_back(entry.first);
  }
  return types;
}

bool zeroCopyPathAvailable()
{
#ifdef ZED_BENCHMARK_HAS_SL_ADAPTER
  return true;
#else
  return false;
#endif
}

TypedSubscription createTypedSubscription(
  rclcpp::Node & node,
  const std::string & topic,
  const std::string & type_name,
  const rclcpp::QoS & qos,
  const rclcpp::SubscriptionOptions & options,
  bool prefer_zero_copy,
  const SampleSink & sink)
{
#ifdef ZED_BENCHMARK_HAS_SL_ADAPTER
  // The adapter's ROS type is sensor_msgs/msg/Image, so an image topic is the
  // only one it can be used on.
  if (prefer_zero_copy && type_name == "sensor_msgs/msg/Image") {
    return makeAdapted(node, topic, qos, options, sink);
  }
#else
  (void)prefer_zero_copy;
#endif

  const auto it = factories().find(type_name);
  if (it == factories().end()) {
    return TypedSubscription();  // unsupported: caller falls back to generic
  }
  return it->second(node, topic, type_name, qos, options, sink);
}

}  // namespace stereolabs
