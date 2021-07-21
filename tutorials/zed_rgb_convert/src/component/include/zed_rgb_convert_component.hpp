/*
 * MIT License
 * 
 * Copyright (c) 2020 Stereolabs
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef ZED_RGB_CONVERT_COMPONENT_HPP
#define ZED_RGB_CONVERT_COMPONENT_HPP

#include "visibility_control.h"

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>


#include <image_transport/image_transport.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/camera_subscriber.hpp>
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
