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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

#include "typed_subscriber.hpp"

#ifdef ZED_BENCHMARK_HAS_THEORA
#include <theora_image_transport/msg/packet.hpp>
#endif
#ifdef ZED_BENCHMARK_HAS_FFMPEG
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#endif
#ifdef ZED_BENCHMARK_HAS_PC_INTERFACES
#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>
#endif

using stereolabs::createTypedSubscription;
using stereolabs::Sample;
using stereolabs::supportedTypedMessageTypes;

namespace
{

/// Spin until `pred` holds or the timeout expires.
template<typename PredT>
bool spinUntil(
  const rclcpp::Node::SharedPtr & node, PredT pred,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(2000))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return pred();
}

}  // namespace

class TypedSubscriberTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

// An unsupported type must be reported as such, so the caller can fall back to
// a generic subscription instead of silently measuring nothing.
TEST_F(TypedSubscriberTest, UnsupportedTypeYieldsNoSubscription) {
  auto node = std::make_shared<rclcpp::Node>("t_unsupported");
  auto result = createTypedSubscription(
    *node, "/whatever", "some_pkg/msg/NotSupported", rclcpp::QoS(1),
    rclcpp::SubscriptionOptions(), false, [](const Sample &) {});
  EXPECT_EQ(result.sub, nullptr);
}

TEST_F(TypedSubscriberTest, RegistryCoversTheZedTopicTypes) {
  const auto types = supportedTypedMessageTypes();
  for (const char * expected :
    {"sensor_msgs/msg/Image", "sensor_msgs/msg/PointCloud2",
      "sensor_msgs/msg/CameraInfo", "sensor_msgs/msg/Imu",
      "sensor_msgs/msg/CompressedImage"})
  {
    EXPECT_NE(
      std::find(types.begin(), types.end(), std::string(expected)),
      types.end()) << "missing " << expected;
  }
}

// Every transport of image_transport / point_cloud_transport that is available
// at build time must be typed, otherwise benchmarking e.g. a theora or a draco
// topic silently falls back to the generic (never intra-process) path.
TEST_F(TypedSubscriberTest, RegistryCoversTheAvailableTransportTypes) {
  const auto types = supportedTypedMessageTypes();
  auto has = [&types](const char * t) {
      return std::find(types.begin(), types.end(), std::string(t)) !=
             types.end();
    };

#ifdef ZED_BENCHMARK_HAS_THEORA
  EXPECT_TRUE(has("theora_image_transport/msg/Packet"));
#endif
#ifdef ZED_BENCHMARK_HAS_FFMPEG
  EXPECT_TRUE(has("ffmpeg_image_transport_msgs/msg/FFMPEGPacket"));
#endif
#ifdef ZED_BENCHMARK_HAS_PC_INTERFACES
  EXPECT_TRUE(has("point_cloud_interfaces/msg/CompressedPointCloud2"));
#endif
  // The sensor_msgs transports need no optional package at all.
  EXPECT_TRUE(has("sensor_msgs/msg/CompressedImage"));
}

#ifdef ZED_BENCHMARK_HAS_THEORA
// A theora topic must report the ogg packet payload, not the fixed fields.
TEST_F(TypedSubscriberTest, TheoraPacketReportsPayloadSize) {
  auto node = std::make_shared<rclcpp::Node>(
    "t_theora", rclcpp::NodeOptions().use_intra_process_comms(true));

  std::vector<Sample> samples;
  auto result = createTypedSubscription(
    *node, "/img/theora", "theora_image_transport/msg/Packet", rclcpp::QoS(1),
    rclcpp::SubscriptionOptions(), false,
    [&samples](const Sample & s) {samples.push_back(s);});
  ASSERT_NE(result.sub, nullptr);

  auto pub = node->create_publisher<theora_image_transport::msg::Packet>(
    "/img/theora", rclcpp::QoS(1));
  auto msg = std::make_unique<theora_image_transport::msg::Packet>();
  msg->data.resize(2048, 0x11);
  msg->header.stamp = node->now();
  pub->publish(std::move(msg));

  ASSERT_TRUE(spinUntil(node, [&samples]() {return !samples.empty();}));
  EXPECT_EQ(samples.front().size_bytes, 2048u);
  EXPECT_TRUE(samples.front().has_stamp);
}
#endif

#ifdef ZED_BENCHMARK_HAS_PC_INTERFACES
// For a compressed cloud the transported volume is the *compressed* payload;
// using the uncompressed size would overstate it by the compression ratio.
TEST_F(TypedSubscriberTest, CompressedPointCloudReportsCompressedSize) {
  auto node = std::make_shared<rclcpp::Node>(
    "t_draco", rclcpp::NodeOptions().use_intra_process_comms(true));

  std::vector<Sample> samples;
  auto result = createTypedSubscription(
    *node, "/cloud/draco", "point_cloud_interfaces/msg/CompressedPointCloud2",
    rclcpp::QoS(1), rclcpp::SubscriptionOptions(), false,
    [&samples](const Sample & s) {samples.push_back(s);});
  ASSERT_NE(result.sub, nullptr);

  auto pub =
    node->create_publisher<point_cloud_interfaces::msg::CompressedPointCloud2>(
    "/cloud/draco", rclcpp::QoS(1));
  auto msg =
    std::make_unique<point_cloud_interfaces::msg::CompressedPointCloud2>();
  msg->compressed_data.resize(777, 0x22);
  // A large uncompressed geometry that must NOT be what gets reported.
  msg->height = 480;
  msg->width = 640;
  msg->row_step = 640 * 16;
  msg->format = "draco";
  msg->header.stamp = node->now();
  pub->publish(std::move(msg));

  ASSERT_TRUE(spinUntil(node, [&samples]() {return !samples.empty();}));
  EXPECT_EQ(samples.front().size_bytes, 777u);
  EXPECT_TRUE(samples.front().has_stamp);
}
#endif

// A plain typed subscription must report the message content size and expose a
// usable stamp, which is what makes the latency measurement possible.
TEST_F(TypedSubscriberTest, TypedSubscriptionReportsSizeAndStamp) {
  auto node = std::make_shared<rclcpp::Node>(
    "t_typed", rclcpp::NodeOptions().use_intra_process_comms(true));

  std::vector<Sample> samples;
  auto result = createTypedSubscription(
    *node, "/cloud", "sensor_msgs/msg/PointCloud2", rclcpp::QoS(1),
    rclcpp::SubscriptionOptions(), false,
    [&samples](const Sample & s) {samples.push_back(s);});
  ASSERT_NE(result.sub, nullptr);
  EXPECT_FALSE(result.zero_copy);

  auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/cloud", rclcpp::QoS(1));

  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  msg->data.resize(4096, 0x5A);
  msg->header.stamp = node->now();
  pub->publish(std::move(msg));

  ASSERT_TRUE(spinUntil(node, [&samples]() {return !samples.empty();}));
  EXPECT_EQ(samples.front().size_bytes, 4096u);
  EXPECT_TRUE(samples.front().has_stamp);
}

// A typed subscription must actually honour a qos_overrides parameter, because
// that is the documented way to change the QoS: rclcpp::create_subscription()
// calls declare_qos_parameters(), so the override reaches the endpoint. The
// generic path cannot do this at all (create_generic_subscription() never reads
// options.qos_overriding_options), which is why the tool warns there instead of
// discarding the request silently.
TEST_F(TypedSubscriberTest, TypedSubscriptionHonoursQosOverrides) {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(
  {
    rclcpp::Parameter(
      "qos_overrides./qos_t.subscription.reliability",
      "reliable")
  });
  auto node = std::make_shared<rclcpp::Node>("t_qos", opts);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();

  auto result = createTypedSubscription(
    *node, "/qos_t", "sensor_msgs/msg/PointCloud2",
    rclcpp::QoS(1).best_effort(), sub_opt, false, [](const Sample &) {});
  ASSERT_NE(result.sub, nullptr);

  // The override must have been declared as a parameter and applied to the
  // endpoint, turning the Best Effort default into Reliable.
  EXPECT_TRUE(node->has_parameter("qos_overrides./qos_t.subscription.reliability"));
  EXPECT_EQ(
    result.sub->get_actual_qos().reliability(),
    rclcpp::ReliabilityPolicy::Reliable);
}

// A zero stamp must not be treated as a timestamp: a latency measured against
// the epoch would be far worse than reporting no latency at all.
TEST_F(TypedSubscriberTest, ZeroStampIsNotReportedAsUsable) {
  auto node = std::make_shared<rclcpp::Node>(
    "t_zero_stamp", rclcpp::NodeOptions().use_intra_process_comms(true));

  std::vector<Sample> samples;
  auto result = createTypedSubscription(
    *node, "/cloud_nostamp", "sensor_msgs/msg/PointCloud2", rclcpp::QoS(1),
    rclcpp::SubscriptionOptions(), false,
    [&samples](const Sample & s) {samples.push_back(s);});
  ASSERT_NE(result.sub, nullptr);

  auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/cloud_nostamp", rclcpp::QoS(1));

  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  msg->data.resize(16);
  // header.stamp deliberately left at zero
  pub->publish(std::move(msg));

  ASSERT_TRUE(spinUntil(node, [&samples]() {return !samples.empty();}));
  EXPECT_FALSE(samples.front().has_stamp);
}

#ifdef ZED_BENCHMARK_HAS_SL_ADAPTER
#include <zed_components/sl_type_adapter.hpp>

using SlImageAdapter =
  rclcpp::TypeAdapter<stereolabs::StampedSlMat, sensor_msgs::msg::Image>;

// The whole point of the type-adapted path: the subscriber must receive the
// publisher's own sl::Mat buffer, not a copy of it. Pointer identity is the
// only assertion that actually proves zero-copy.
//
// It also proves intra-process delivery on its own: StampedSlMat is a custom
// C++ type with no wire representation, so it can only ever be handed over
// in-process.
TEST_F(TypedSubscriberTest, AdaptedPathIsTrulyZeroCopy) {
  ASSERT_TRUE(stereolabs::zeroCopyPathAvailable());

  auto node = std::make_shared<rclcpp::Node>(
    "t_zero_copy", rclcpp::NodeOptions().use_intra_process_comms(true));

  std::vector<Sample> samples;
  auto result = createTypedSubscription(
    *node, "/img", "sensor_msgs/msg/Image", rclcpp::QoS(1),
    rclcpp::SubscriptionOptions(), /*prefer_zero_copy=*/ true,
    [&samples](const Sample & s) {samples.push_back(s);});
  ASSERT_NE(result.sub, nullptr);
  EXPECT_TRUE(result.zero_copy);

  // A second, independent adapted subscription observes the delivered pointer,
  // which the Sample intentionally does not carry.
  const uint8_t * rx_ptr = nullptr;
  auto observer = node->create_subscription<SlImageAdapter>(
    "/img", rclcpp::QoS(1),
    [&rx_ptr](std::shared_ptr<const stereolabs::StampedSlMat> m) {
      rx_ptr = const_cast<sl::Mat &>(m->mat).getPtr<sl::uchar1>(sl::MEM::CPU);
    });

  auto pub = node->create_publisher<SlImageAdapter>("/img", rclcpp::QoS(1));

  auto msg = std::make_unique<stereolabs::StampedSlMat>();
  msg->mat.alloc(sl::Resolution(64, 48), sl::MAT_TYPE::U8_C1, sl::MEM::CPU);
  msg->frame_id = "test";
  msg->stamp = node->now();
  const uint8_t * tx_ptr = msg->mat.getPtr<sl::uchar1>(sl::MEM::CPU);
  const size_t expected_bytes = msg->mat.getStepBytes() * msg->mat.getHeight();
  pub->publish(std::move(msg));

  ASSERT_TRUE(
    spinUntil(
      node, [&samples, &rx_ptr]() {
        return !samples.empty() && rx_ptr != nullptr;
      }));

  EXPECT_EQ(rx_ptr, tx_ptr) << "the buffer was copied: not zero-copy";
  EXPECT_EQ(samples.front().size_bytes, expected_bytes);
  EXPECT_TRUE(samples.front().has_stamp);
}

// Without prefer_zero_copy an image topic must use the plain typed path, so a
// caller that has not enabled intra-process comms is not silently handed a
// subscription that only an intra-process publisher can feed.
TEST_F(TypedSubscriberTest, ImageFallsBackToPlainTypedWhenZeroCopyNotWanted) {
  auto node = std::make_shared<rclcpp::Node>("t_plain_img");
  auto result = createTypedSubscription(
    *node, "/img_plain", "sensor_msgs/msg/Image", rclcpp::QoS(1),
    rclcpp::SubscriptionOptions(), /*prefer_zero_copy=*/ false,
    [](const Sample &) {});
  ASSERT_NE(result.sub, nullptr);
  EXPECT_FALSE(result.zero_copy);
}
#endif  // ZED_BENCHMARK_HAS_SL_ADAPTER
