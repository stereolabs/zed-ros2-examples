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
 * This tutorial demonstrates simple receipt of ZED depth messages over the ROS system.
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::placeholders;

class MinimalDepthSubscriber : public rclcpp::Node {
  public:
    MinimalDepthSubscriber()
        : Node("zed_depth_tutorial") {

        /* Note: it is very important to use a QOS profile for the subscriber that is compatible
         * with the QOS profile of the publisher.
         * The ZED component node uses a default QoS profile with reliability set as "RELIABLE"
         * and durability set as "VOLATILE".
         * To be able to receive the subscribed topic the subscriber must use compatible
         * parameters.
         */

        // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings

        rclcpp::QoS depth_qos(10);
        depth_qos.keep_last(10);
        depth_qos.best_effort();
        depth_qos.durability_volatile();

        // Create depth map subscriber
        mDepthSub = create_subscription<sensor_msgs::msg::Image>(
                   "depth", depth_qos,
                   std::bind(&MinimalDepthSubscriber::depthCallback, this, _1) );
    }

  protected:
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Get a pointer to the depth values casting the data
        // pointer to floating point
        float* depths = (float*)(&msg->data[0]);

        // Image coordinates of the center pixel
        int u = msg->width / 2;
        int v = msg->height / 2;

        // Linear index of the center pixel
        int centerIdx = u + msg->width * v;

        // Output the measure
        RCLCPP_INFO(get_logger(), "Center distance : %g m", depths[centerIdx]);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mDepthSub;
};

// The main function
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto depth_node = std::make_shared<MinimalDepthSubscriber>();

    rclcpp::spin(depth_node);
    rclcpp::shutdown();
    return 0;
}
