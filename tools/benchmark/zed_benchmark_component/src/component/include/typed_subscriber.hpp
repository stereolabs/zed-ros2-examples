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

#ifndef TYPED_SUBSCRIBER_HPP_
#define TYPED_SUBSCRIBER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace stereolabs
{

/// @brief How the message size reported by a subscription must be interpreted.
///
/// The two are deliberately NOT interchangeable, and the report says which one
/// it is showing: a bandwidth computed from wire bytes and one computed from
/// message content are not comparable figures.
enum class SizeSemantics
{
  /// Exact serialized (on-the-wire) byte count. Only a generic subscription
  /// sees this, because only it receives the serialized buffer.
  SerializedWire,
  /// Message content bytes: variable-length data plus fixed numeric fields,
  /// excluding CDR framing and padding. This is what a typed subscription can
  /// account for without paying for a serialization it never performed.
  MessageContent
};

/// @brief A single measurement, produced by any subscription path.
struct Sample
{
  /// Message size, to be read according to the active SizeSemantics.
  size_t size_bytes = 0;
  /// True when `stamp` carries a usable publisher-side timestamp.
  bool has_stamp = false;
  /// Publisher-side timestamp, used to derive the end-to-end latency.
  rclcpp::Time stamp;
  /// True only when this very message is *known* to have been delivered
  /// intra-process. Set by the type-adapted path, where receiving the custom
  /// C++ type is itself proof: that type has no wire representation, so it
  /// cannot have come through the middleware. It stays false on the plain typed
  /// path, where the two are indistinguishable from inside the callback - so a
  /// report must never claim intra-process delivery on the strength of the node
  /// merely having intra-process comms enabled.
  bool intra_process_confirmed = false;
};

/// @brief Sink handed every measurement, whatever the subscription path.
using SampleSink = std::function<void (const Sample &)>;

/// @brief Outcome of a typed-subscription attempt.
struct TypedSubscription
{
  /// The created subscription. Null when the message type is not supported,
  /// in which case the caller must fall back to a generic subscription.
  rclcpp::SubscriptionBase::SharedPtr sub;
  /// True only for the ZED type-adapted path, which receives the publisher's
  /// own buffer by pointer. A plain typed subscription is intra-process
  /// capable but rclcpp still copies the message into it.
  bool zero_copy = false;
  /// Short human-readable description of the path, used in logs and reports.
  std::string description;
};

/// @brief ROS message type names a typed subscription can be built for.
std::vector<std::string> supportedTypedMessageTypes();

/// @brief Whether this build includes the ZED type-adapted zero-copy path.
///
/// The path is compiled in only when `zed_components` and the ZED SDK are
/// available at build time, so the benchmark stays usable without them.
bool zeroCopyPathAvailable();

/// @brief Try to build a typed subscription for `type_name`.
///
/// A typed subscription is the only kind that can take the intra-process path:
/// rclcpp registers a subscription with the IntraProcessManager solely from the
/// constructor of the templated rclcpp::Subscription<T>.
///
/// @param prefer_zero_copy When true and the topic type has a ZED TypeAdapter,
///        subscribe through the adapter to receive the publisher's buffer
///        without any copy.
/// @return A TypedSubscription with a null `sub` when `type_name` is not
///         supported.
TypedSubscription createTypedSubscription(
  rclcpp::Node & node,
  const std::string & topic,
  const std::string & type_name,
  const rclcpp::QoS & qos,
  const rclcpp::SubscriptionOptions & options,
  bool prefer_zero_copy,
  const SampleSink & sink);

}  // namespace stereolabs

#endif  // TYPED_SUBSCRIBER_HPP_
