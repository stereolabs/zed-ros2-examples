#include "yolo_detector.hpp"

namespace stereolabs
{
ZedYoloDetector::ZedYoloDetector(const rclcpp::NodeOptions & options)
: ZedCustomOd(options)
{
  RCLCPP_INFO(get_logger(), "***********************************");
  RCLCPP_INFO(get_logger(), " ZED Yolo detector Component ");
  RCLCPP_INFO(get_logger(), "************************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "***********************************");

  // Initialize the custom detector
  init();
}

void ZedYoloDetector::readParams()
{
  RCLCPP_INFO(get_logger(), "*** YOLO Detector Parameters ***");


}

void ZedYoloDetector::init()
{
  readParams();
}

void ZedYoloDetector::doInference()
{
  RCLCPP_INFO_STREAM(get_logger(), "ZedYoloDetector::doInference()");
}

} // namespace stereolabs

// *************************************************************************
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedYoloDetector)
// *************************************************************************
