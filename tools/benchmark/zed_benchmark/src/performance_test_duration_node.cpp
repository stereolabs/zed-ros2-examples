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

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>


/**
 * @brief Timer-based shutdown node for ROS 2 performance testing.
 *
 * This node starts a wall timer upon initialization and monitors its elapsed
 * time against a user-defined target duration. Once the target time is reached,
 * the node initiates a clean shutdown of the ROS 2 context.
 *
 * In the ROS 2 ecosystem, this node is launched as a *required* node within the
 * full wrapper performance test launcher. As a result, when this node shuts
 * down, it terminates the launcher and all associated component nodes,
 * effectively marking the end of the performance test and triggering the
 * collection of test results.
 *
 * This design ensures that all tests run for a deterministic duration and
 * complete in a fully synchronized manner.
 */
class PerformanceTestDurationNode : public rclcpp::Node
{
public:
    PerformanceTestDurationNode() : Node("performance_test_duration_node")
    {
        // Declare a parameter with a default value of 120 seconds
        this->declare_parameter("performance_test_duration", 60);
        
        // Get the parameter value
        performance_test_duration_ = this->get_parameter("performance_test_duration").as_int();

        RCLCPP_INFO(this->get_logger(), "Node has started. Will exit after %d seconds...", performance_test_duration_);

        // Sleep for the configured duration
        std::this_thread::sleep_for(std::chrono::seconds(performance_test_duration_));

        RCLCPP_INFO(this->get_logger(), "Node is shutting down.");
    }

private:
    int performance_test_duration_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PerformanceTestDurationNode>();
    rclcpp::shutdown();
    return 0;
}

