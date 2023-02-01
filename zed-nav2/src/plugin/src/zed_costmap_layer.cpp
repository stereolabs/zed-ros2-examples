#include <string>

#include <nav2_costmap_2d/costmap_math.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <rclcpp/parameter_events_filter.hpp>

#include <grid_map_ros/grid_map_ros.hpp>

#include "zed_costmap_layer.hpp"

namespace zed_nav2
{

ZedCostmapLayer::ZedCostmapLayer() {}

void ZedCostmapLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  enabled_ = node->declare_parameter(name_ + "." + "enabled", true);

  // Get the path of the map slice topic.
  std::string grid_map_topic = "/local_map/gridmap";

  grid_map_topic = node->declare_parameter<std::string>(
    getFullName("grid_map_topic"), grid_map_topic);
  max_cost_value_ = node->declare_parameter<uint8_t>(
    getFullName("max_cost_value"), max_cost_value_);

  RCLCPP_INFO_STREAM(
    node->get_logger(),
    "Name: " << name_ << " Topic name: " << grid_map_topic <<
      " max_cost_value: " << max_cost_value_);

  // Add subscribers to the gridmap message.
  map_sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    grid_map_topic, 1,
    std::bind(
      &ZedCostmapLayer::gridmapCallback, this,
      std::placeholders::_1));
  current_ = true;
}

// The method is called to ask the plugin: which area of costmap it needs to
// update.
void ZedCostmapLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/,
  double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x,
  double * max_y)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  /*if (map_.exists("elevation") && map_.exists("occupancy") ) {
    *min_x = -map_.getLength().x() / 2.0;
    *max_x = map_.getLength().x() / 2.0;
    *min_y = -map_.getLength().y() / 2.0;
    *max_y = map_.getLength().y() / 2.0;
  } else {
    RCLCPP_WARN(
      node->get_logger(),
      "The gridmap message does not contain the required 'elevation' and 'occupancy' layers");
  }*/

  RCLCPP_DEBUG(
    node->get_logger(),
    "Update bounds: Min x: %f Min y: %f Max x: %f Max y: %f", *min_x,
    *min_y, *max_x, *max_y);
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
void ZedCostmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  RCLCPP_DEBUG(
    node->get_logger(),
    "Update costs: Min i: %d Min j: %d Max i: %d Max j: %d", min_i,
    min_j, max_i, max_j);

  setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
  matchSize();
  uint8_t * costmap_array = getCharMap();
  unsigned int size_x = getSizeInCellsX(), size_y = getSizeInCellsY();

  RCLCPP_DEBUG(
    node->get_logger(), "Size in cells x: %d size in cells y: %d",
    size_x, size_y);

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = getIndex(i, j);

      // Figure out the world coordinates of this gridcell.
      //double world_x, world_y;
      //mapToWorld(i, j, world_x, world_y);

      // Look up the corresponding cell in our latest map.
      //float value = map_.atPosition("occupancy", grid_map::Position(world_x, world_y));
      float value = map_.at("occupancy", grid_map::Index(i, j));

      // Calculate what this maps to in the original structure.
      uint8_t cost = nav2_costmap_2d::NO_INFORMATION;

      // ----> From ZED SDK
      const float UNKNOWN_CELL = -1.f;
      // values for OCCUPANCY
      const float FREE_CELL = 0.f;
      const float OCCUPIED_CELL = 100.f;
      // <---- From ZED SDK

      // Convert the distance value to a costmap value.
      if (value == FREE_CELL) {
        cost = nav2_costmap_2d::FREE_SPACE;
      } else if (value == OCCUPIED_CELL) {
        cost = nav2_costmap_2d::LETHAL_OBSTACLE;
      } else {
        cost = static_cast<uint8_t>(
          max_cost_value_ *
          (1.0f - std::min<float>(
            value / max_obstacle_distance_,
            1.0f)));
      }

      costmap_array[index] = cost;
    }
  }

  // This combines the master costmap with the current costmap by taking
  // the max across all costmaps.
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  RCLCPP_DEBUG(node->get_logger(), "Finished updating.");
  current_ = true;
}

void ZedCostmapLayer::gridmapCallback(
  const grid_map_msgs::msg::GridMap::ConstSharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  RCLCPP_DEBUG(node->get_logger(), "GridMap callback.");

  grid_map::GridMapRosConverter::fromMessage(*msg, map_);
}

}  // namespace zed_nav2

// Register the macro for this layer
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(zed_nav2::ZedCostmapLayer, nav2_costmap_2d::Layer)
