// Copyright 2022 Stereolabs
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

#ifndef ZED_COSTMAP_LAYER_HPP_
#define ZED_COSTMAP_LAYER_HPP_

#define RGB_LAYER "rgb"
#define TRAVERSABILITY_LAYER "traversability"
#define ELEVATION_LAYER "elevation"

#include "visibility_control.hpp"

#include <Eigen/Core>
#include <string>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

namespace zed_nav2
{

  class ZedCostmapLayer : public nav2_costmap_2d::CostmapLayer
  {
  public:
    ZED_COSTMAP_2D_PLUGIN_PUBLIC
    explicit ZedCostmapLayer();

    void onInitialize() override;
    void updateBounds(
        double robot_x, double robot_y, double robot_yaw,
        double *min_x, double *min_y, double *max_x,
        double *max_y) override;
    void updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid, int min_i,
        int min_j, int max_i, int max_j) override;

    void reset() override {}
    bool isClearable() override { return true; }

    void gridmapCallback(
        const grid_map_msgs::msg::GridMap::ConstSharedPtr msg);

  private:
    bool debug_;

    // ----> Parameters
    float max_obstacle_distance_ = 1.0f;
    float inflation_distance_ = 0.5f;
    float max_traversability_cost_ = 0.5f;
    std::string target_frame_id_ = "";
    //  <---- Parameters

    // ----> Grid map
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr
        map_sub_;
    grid_map::GridMap map_;
    // This should not include any "special" values like 255.
    float max_cost_value_ = 252;
    std::mutex mGrid_mutex;
    // <---- Grid map

    // ----> initialization Transform listener
    std::unique_ptr<tf2_ros::Buffer> mTfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> mTfListener;
    // <---- initialization Transform listener

    bool checkLayersAndWarn();
  };

} // namespace zed_nav2

#endif // ZED_COSTMAP_LAYER_HPP_
