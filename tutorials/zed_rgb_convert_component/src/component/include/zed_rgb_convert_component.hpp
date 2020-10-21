#ifndef ZED_RGB_CONVERT_COMPONENT_HPP
#define ZED_RGB_CONVERT_COMPONENT_HPP

#include "visibility_control.h"

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>


#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/camera_subscriber.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace stereolabs {

class ZedRgbCvtComponent : public rclcpp::Node
{

public:
    ZED_CVT_COMPONENT_PUBLIC
    explicit ZedRgbCvtComponent(const rclcpp::NodeOptions & options);

    virtual ~ZedRgbCvtComponent(){}

protected:
    void camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info);

private:
    // Publisher
    image_transport::CameraPublisher mPubBgr;

    // Subscriber
    image_transport::CameraSubscriber mSubBgra;

    // QoS parameters
    rclcpp::QoS mVideoQos;
};

} // namespace stereolabs

#endif // ZED_RGB_CONVERT_COMPONENT_HPP
