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

#include "zed_nitros_sub_component.hpp"

namespace stereolabs
{

ZedNitrosSubComponent::ZedNitrosSubComponent(const rclcpp::NodeOptions & options)
: Node("zed_nitros_sub_component", options), _defaultQoS(10)
{
  RCLCPP_INFO(get_logger(), "=================================");
  RCLCPP_INFO(get_logger(), " ZED Nitros Subscriber Component");
  RCLCPP_INFO(get_logger(), "=================================");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(
    get_logger(), " * IPC: %s",
    options.use_intra_process_comms() ? "enabled" : "disabled");
  RCLCPP_INFO(get_logger(), "================================");

  // Read the parameters from the parameter server
  read_parameters();

  // Start the example and perform the benchmark
  create_std_subscriber();
}

void ZedNitrosSubComponent::read_parameters()
{
  RCLCPP_INFO(get_logger(), "--------------------");
  RCLCPP_INFO(get_logger(), " Example Parameters");
  RCLCPP_INFO(get_logger(), "--------------------");

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;

  std::string paramName = "";

  // Number of samples to process before exiting
  descriptor.description = "Total number of samples to process before exiting";
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = 1;
  range.to_value = 1000;
  descriptor.integer_range.push_back(range);
  paramName = "test.tot_samples";
  this->declare_parameter(paramName, rclcpp::ParameterValue(_totSamples), descriptor);
  if (!this->get_parameter(paramName, _totSamples)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << _totSamples);
  }
  RCLCPP_INFO_STREAM(get_logger(), " * " << paramName << ": " << _totSamples);

  // Isaac ROS Nitros debug information
  descriptor.description = "Isaac ROS Nitros debug information";
  paramName = "debug.debug_nitros";
  this->declare_parameter(paramName, rclcpp::ParameterValue(_debugNitros), descriptor);
  if (!this->get_parameter(paramName, _debugNitros)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << _totSamples);
  }
  RCLCPP_INFO_STREAM(get_logger(), " * " << paramName << ": " << (_debugNitros ? "TRUE" : "FALSE"));
  if (_debugNitros) {
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);
  }
}

void ZedNitrosSubComponent::create_std_subscriber()
{
  // Create a subscriber to the image topic
  _sub = this->create_subscription<sensor_msgs::msg::Image>(
    "image", _defaultQoS,
    std::bind(
      &ZedNitrosSubComponent::std_sub_callback, this,
      std::placeholders::_1));
}

void ZedNitrosSubComponent::create_nitros_subscriber()
{
  // Create a Nitros subscriber to the Nitros topic
  _nitrosSub = std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
        nvidia::isaac_ros::nitros::NitrosImageView>>(
    this, "image",
    _isDepth ? nvidia::isaac_ros::nitros::nitros_image_32FC1_t::supported_type_name :
    nvidia::isaac_ros::nitros::nitros_image_rgba8_t::supported_type_name,
    std::bind(
      &ZedNitrosSubComponent::nitros_sub_callback, this,
      std::placeholders::_1));
}

void ZedNitrosSubComponent::std_sub_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & img)
{
  auto now = this->now();

  static bool first_frame = true;
  if (first_frame) {
    _encoding = img->encoding;
    first_frame = false;
    if (img->encoding == "32FC1") {

      _isDepth = true;
      RCLCPP_DEBUG(this->get_logger(), "Receiving Depth map messages.");
    } else if (img->encoding == "bgra8") {
      _isDepth = false;
      RCLCPP_DEBUG(this->get_logger(), "Receiving RGB image messages.");
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Received an unsupported image encoding: %s. Only BGRA and 32FC1 are supported.",
        img->encoding.c_str());
      rclcpp::shutdown();
    }
  }

  // Check example coherency
  if (img->encoding != _encoding) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Topic type has changed from %s to %s. This is not supported. Exiting.",
      _encoding.c_str(), img->encoding.c_str());
    rclcpp::shutdown();
  }

  RCLCPP_INFO(this->get_logger(), "Receiving Image buffer with memory at: %p", img->data.data());
  RCLCPP_INFO(
    this->get_logger(), " * encoding: %s", img->encoding.c_str());
  RCLCPP_INFO(
    this->get_logger(), " * width: %d, height: %d", img->width, img->height);

  rclcpp::Time img_ts;
  int32_t ts_sec = img->header.stamp.sec;
  int32_t ts_nsec = img->header.stamp.nanosec;
  img_ts = rclcpp::Time(ts_sec, ts_nsec, now.get_clock_type());

  RCLCPP_INFO(
    this->get_logger(), " * Latency: %f sec",
    now.seconds() - img_ts.seconds());

  // Check if we acquired the desired number of samples
  static int std_sample_count = 0;
  std_sample_count++;
  RCLCPP_INFO(
    this->get_logger(), " *** Sample %d/%d ***", std_sample_count, _totSamples);

  if (std_sample_count >= _totSamples) {
    RCLCPP_INFO(this->get_logger(), "Received %d standard samples. Unsubscribing...", _totSamples);
    _sub.reset();
    RCLCPP_INFO(this->get_logger(), "Standard subscriber unsubscribed.");

    // Create and enable the Nitros subscriber
    create_nitros_subscriber();
  }
}

void ZedNitrosSubComponent::nitros_sub_callback(
  const nvidia::isaac_ros::nitros::NitrosImageView & img)
{
  // Check example coherency
  if (img.GetEncoding() != _encoding) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Topic type has changed from %s to %s. This is not supported. Exiting.",
      _encoding.c_str(), img.GetEncoding().c_str());
    rclcpp::shutdown();
  }

  auto now = this->now();

  RCLCPP_INFO(this->get_logger(), "Receiving CUDA buffer with memory at: %p", img.GetGpuData());
  RCLCPP_INFO(
    this->get_logger(),
    " * encoding: %s", img.GetEncoding().c_str());
  RCLCPP_INFO(
    this->get_logger(),
    " * width: %d, height: %d",
    img.GetWidth(), img.GetHeight());

  rclcpp::Time img_ts;
  int32_t ts_sec = img.GetTimestampSeconds();
  int32_t ts_nsec = img.GetTimestampNanoseconds();
  img_ts = rclcpp::Time(ts_sec, ts_nsec, now.get_clock_type());
  RCLCPP_INFO(this->get_logger(), " * Latency: %f sec", now.seconds() - img_ts.seconds());

  // Check if we acquired the desired number of samples
  static int nitros_sample_count = 0;
  nitros_sample_count++;
  RCLCPP_INFO(
    this->get_logger(), " *** Sample %d/%d ***", nitros_sample_count, _totSamples);

  if (nitros_sample_count >= _totSamples) {
    RCLCPP_INFO(this->get_logger(), "Received %d Nitros samples. Unsubscribing...", _totSamples);
    _nitrosSub.reset();
    RCLCPP_INFO(this->get_logger(), "Nitros subscriber unsubscribed.");

    // Calculate the statistics
    RCLCPP_INFO(this->get_logger(), "Example completed.");
    RCLCPP_INFO(this->get_logger(), "-----------------------------------");

    // Perform a clean shutdown of the node
    RCLCPP_INFO(this->get_logger(), "Shutting down the node...");
    rclcpp::shutdown();
  }
}

}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedNitrosSubComponent)
