///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/**
 * This tutorial demonstrates simple receipt of ZED video messages over the ROS system.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>

rclcpp::Node::SharedPtr g_node = nullptr;

/**
 * Subscriber callbacks. The argument of the callback is a constant pointer to the received message
 */


void imageRightRectifiedCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(g_node->get_logger(),
                "Right Rectified image received from ZED\tSize: %dx%d - Timestamp: %u.%u sec ",
                msg->width, msg->height,
                msg->header.stamp.sec,msg->header.stamp.nanosec);
}

void imageLeftRectifiedCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(g_node->get_logger(),
                "Left  Rectified image received from ZED\tSize: %dx%d - Timestamp: %u.%u sec ",
                msg->width, msg->height,
                msg->header.stamp.sec,msg->header.stamp.nanosec);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create the node
    g_node = rclcpp::Node::make_shared("zed_video_tutorial");


    /* Note: it is very important to use a QOS profile for the subscriber that is compatible
     * with the QOS profile of the publisher.
     * The ZED component node uses a default QoS profile with reliability set as "RELIABLE"
     * and durability set as "VOLATILE".
     * To be able to receive the subscribed topic the subscriber must use compatible
     * parameters.
     */

    // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings

    rclcpp::QoS video_qos(10);
    video_qos.keep_last(10);
    video_qos.best_effort();
    video_qos.durability_volatile();

    // Create right image subscriber
    auto right_sub = g_node->create_subscription<sensor_msgs::msg::Image>(
                "right_image", video_qos, imageRightRectifiedCallback );

    // Create left image subscriber
    auto left_sub = g_node->create_subscription<sensor_msgs::msg::Image>(
                "left_image", video_qos, imageLeftRectifiedCallback );

    // Let the node run
    rclcpp::spin(g_node);

    // Shutdown when the node is stopped using Ctrl+C
    rclcpp::shutdown();

    return 0;
}
