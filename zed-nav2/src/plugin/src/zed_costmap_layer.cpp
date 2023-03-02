#include <string>
#include <stdexcept>
#include <limits>
#include <sstream>

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
    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }
    enabled_ = node->declare_parameter<bool>(getFullName("enabled"), enabled_);
    debug_ = node->declare_parameter<bool>(getFullName("debug"), debug_);
    target_frame_id_ = node->declare_parameter<std::string>(getFullName("target_frame_id"), target_frame_id_);
    max_traversability_cost_ = node->declare_parameter<float>(getFullName("max_traversability_cost"), max_traversability_cost_);

    // Get the path of the gridmap topic.
    std::string grid_map_topic = "/local_map/gridmap";

    grid_map_topic = node->declare_parameter<std::string>(
        getFullName("grid_map_topic"), grid_map_topic);
    // max_cost_value_ = node->declare_parameter<uint8_t>(
    //     getFullName("max_cost_value"), max_cost_value_);

    if (debug_)
    {
      rcutils_ret_t res =
          rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

      if (res != RCUTILS_RET_OK)
      {
        RCLCPP_INFO_STREAM(
            node->get_logger(),
            "Name: " << name_ << ": Error setting DEBUG level for logger");
      }
      else
      {
        RCLCPP_INFO_STREAM(
            node->get_logger(),
            "Name: " << name_ << ": Debug Mode enabled");
      }
    }
    else
    {
      rcutils_ret_t res =
          rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

      if (res != RCUTILS_RET_OK)
      {
        RCLCPP_INFO_STREAM(
            node->get_logger(),
            "Name: " << name_ << ": Error setting INFO level for logger");
      }
    }

    RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Name: " << name_ << " - Topic name: " << grid_map_topic);

    RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Name: " << name_ << " - Target frame ID: " << target_frame_id_);

    RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Name: " << name_ << " - Max traversability cost: " << max_traversability_cost_);

    // ----> TF2 Transform
    mTfBuffer = std::make_unique<tf2_ros::Buffer>(clock_);
    mTfListener = std::make_unique<tf2_ros::TransformListener>(*mTfBuffer); // Start TF Listener thread

    // <---- TF2 Transform

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
      double robot_x, double robot_y,
      double robot_yaw, double *min_x,
      double *min_y, double *max_x,
      double *max_y)
  {
    auto node = node_.lock();
    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }

    if (map_.exists(ELEVATION_LAYER) && map_.exists(RGB_LAYER) && map_.exists(TRAVERSABILITY_LAYER))
    {
      *min_x = -map_.getLength().x() / 2.0;
      *max_x = map_.getLength().x() / 2.0;
      *min_y = -map_.getLength().y() / 2.0;
      *max_y = map_.getLength().y() / 2.0;
    }
    else
    {
      RCLCPP_WARN(
          node->get_logger(),
          "The gridmap message does not contain the required '%s', '%s', and '%s' layers",
          ELEVATION_LAYER,
          RGB_LAYER,
          TRAVERSABILITY_LAYER);
      std::vector<std::string> layers = map_.getLayers();
      std::stringstream ss;
      for (size_t s = 0; s < layers.size(); s++)
      {
        ss << layers[s];
        ss << ",";
      }
      RCLCPP_DEBUG_STREAM(
          node->get_logger(),
          "Gridmap not valid. Available layers: " << ss.str().c_str());

      return;
    }

    RCLCPP_DEBUG(
        node->get_logger(),
        "Update bounds: Robot X: %f Robot Y: %f Robot Yaw: %f Min x: %f Min y: %f Max x: %f Max y: %f", robot_x, robot_y, robot_yaw, *min_x,
        *min_y, *max_x, *max_y);
  }

  // The method is called when costmap recalculation is required.
  // It updates the costmap within its window bounds.
  void ZedCostmapLayer::updateCosts(
      nav2_costmap_2d::Costmap2D &master_grid,
      int min_i, int min_j, int max_i,
      int max_j)
  {
    if (!enabled_)
    {
      return;
    }
    mGrid_mutex.lock();
    auto node = node_.lock();
    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }

    RCLCPP_DEBUG(
        node->get_logger(),
        "Update costs: Min i: %d Min j: %d Max i: %d Max j: %d", min_i,
        min_j, max_i, max_j);

    setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
    matchSize();
    uint8_t *costmap_array = master_grid.getCharMap();
    unsigned int size_x = getSizeInCellsX(), size_y = getSizeInCellsY();

    RCLCPP_DEBUG(
        node->get_logger(), "Size in cells x: %d size in cells y: %d",
        size_x, size_y);

    if (!map_.exists(ELEVATION_LAYER) || !map_.exists(RGB_LAYER) || !map_.exists(TRAVERSABILITY_LAYER))
    {
      std::vector<std::string> layers = map_.getLayers();
      std::stringstream ss;
      for (size_t s = 0; s < layers.size(); s++)
      {
        ss << layers[s];
        ss << ",";
      }
      RCLCPP_DEBUG_STREAM(
          node->get_logger(),
          "Gridmap not valid. Available layers: " << ss.str().c_str());
    }

    for (int j = min_j; j < max_j; j++)
    {
      for (int i = min_i; i < max_i; i++)
      {
        int index = getIndex(i, j);

        // Figure out the world coordinates of this gridcell.
        // double world_x, world_y;
        // mapToWorld(i, j, world_x, world_y);

        // Look up the corresponding cell in our latest map.
        // float value = map_.atPosition("occupancy", grid_map::Position(world_x, world_y));
        // float occupancy_value = std::numeric_limits<double>::quiet_NaN();
        float traversability_value = std::numeric_limits<double>::quiet_NaN();
        try
        {
          // occupancy_value = map_.at(RGB_LAYER, grid_map::Index(i, j));
          auto size = map_.getSize();
          auto xx = size[0];
          auto yy = size[1];
          RCLCPP_DEBUG(logger_, "Size x: %d, Size y: %d", xx, yy);
          auto layers = map_.getBasicLayers();
          auto idx = grid_map::Index(i, j);
          traversability_value = map_.at(TRAVERSABILITY_LAYER, grid_map::Index(j, i));
        }
        catch (const std::out_of_range &e)
        {
          RCLCPP_DEBUG_STREAM(
              node->get_logger(),
              "Gridmap exception for index (" << i << "," << j << "): " << e.what());
          RCLCPP_DEBUG_STREAM(
              node->get_logger(),
              "Gridmap size: " << map_.getLength().x() / map_.getResolution() << "x" << map_.getLength().y() / map_.getResolution());
        }
        catch (const std::exception &ex)
        {
          RCLCPP_ERROR(logger_, "I go boom boom because: %s", ex.what());
        }

        // Calculate what this maps to in the original structure.
        uint8_t cost = nav2_costmap_2d::NO_INFORMATION;

        // ----> From ZED SDK
        const float UNKNOWN_CELL = -1.f;
        // values for OCCUPANCY
        const float FREE_CELL = 0.f;
        const float OCCUPIED_CELL = 1.f;
        // <---- From ZED SDK

        // Convert the distance value to a costmap value.
        if (traversability_value == FREE_CELL)
        {
          cost = nav2_costmap_2d::FREE_SPACE;
        }
        else if (traversability_value >= max_traversability_cost_ || traversability_value == OCCUPIED_CELL)
        {
          cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        }
        else
        {
          cost = static_cast<uint8_t>(max_cost_value_ * traversability_value); // traversability is between 0.0 and 1.0
        }
        RCLCPP_DEBUG(logger_, "Cost at cell %d %d is: %d.", i, j, cost);

        // Reverse cell order because of different conventions between Costmap and grid map.
        size_t updateRange = (max_j - min_j) * (max_i - min_i);
        int upper_bound = getIndex(max_i, max_j);
        int lower_bound = getIndex(min_i, min_j);
        int reversed_index = upper_bound - 1 - index + lower_bound;
        if (reversed_index >= lower_bound && reversed_index < upper_bound)
        {
          costmap_array[reversed_index] = cost;
          RCLCPP_DEBUG(logger_, "Updated cost at cell %d %d is: %d.", i, j, costmap_array[reversed_index]);
        }
        else
        {
          RCLCPP_DEBUG(logger_, "Out of bounds reversed index %d %d is: %d.", i, j, reversed_index);
        }
      }
    }

    // This combines the master costmap with the current costmap by taking
    // the max across all costmaps.
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    RCLCPP_DEBUG(node->get_logger(), "Finished updating.");
    mGrid_mutex.unlock();
    current_ = true;
  }

  void ZedCostmapLayer::gridmapCallback(
      const grid_map_msgs::msg::GridMap::ConstSharedPtr msg)
  {
    auto node = node_.lock();
    static bool first_tf_error = true;

    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }

    RCLCPP_DEBUG(node->get_logger(), "GridMap callback.");

    // Deserialize into grid map
    mGrid_mutex.lock();
    grid_map::GridMapRosConverter::fromMessage(*msg, map_);

    mGrid_mutex.unlock();

    // if (msg->header.frame_id != target_frame_id_)
    // {
    //   RCLCPP_DEBUG(node->get_logger(), "Transforming map from %s to %s.", msg->header.frame_id.c_str(), target_frame_id_.c_str());
    //   try
    //   {
    //     auto tf_stamped = mTfBuffer->lookupTransform(target_frame_id_,
    //                                                  msg->header.frame_id,
    //                                                  rclcpp::Time(0, 0, RCL_ROS_TIME),
    //                                                  rclcpp::Duration(1, 0));

    //     // Check if it is only a transaltion - the call to move could be less expensive
    //     // if (tf_stamped.transform.rotation.w == 1.0 &&
    //     //     tf_stamped.transform.rotation.x == 0.0 &&
    //     //     tf_stamped.transform.rotation.y == 0.0 &&
    //     //     tf_stamped.transform.rotation.z == 0.0)
    //     // {
    //     //   auto transaltion = grid_map::Position(tf_stamped.transform.translation.x,
    //     //                                         tf_stamped.transform.translation.y);
    //     //   RCLCPP_DEBUG(logger_, "Applying Translation");
    //     //   map_.move(transaltion);
    //     //   RCLCPP_DEBUG(logger_, "Transform applied");
    //     // }
    //     // else
    //     {
    //       auto quat = Eigen::Quaternion<double>(tf_stamped.transform.rotation.w,
    //                                             tf_stamped.transform.rotation.x,
    //                                             tf_stamped.transform.rotation.y,
    //                                             tf_stamped.transform.rotation.z);
    //       auto translation = Eigen::Vector3d(tf_stamped.transform.translation.x,
    //                                          tf_stamped.transform.translation.y,
    //                                          tf_stamped.transform.translation.z);
    //       RCLCPP_DEBUG(logger_, "Applying transform");
    //       Eigen::Isometry3d transform;
    //       transform.rotate(quat);
    //       transform.translate(translation);
    //       map_ = map_.getTransformedMap(transform, ELEVATION_LAYER, target_frame_id_);
    //       RCLCPP_DEBUG(logger_, "Transform applied");
    //     }
    //     // Apply frame conversion to grid map
    //   }
    //   catch (tf2::TransformException &ex)
    //   {
    //     if (!first_tf_error)
    //     {
    //       rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    //       RCLCPP_WARN_THROTTLE(logger_, steady_clock, 1.0, "Transform error: %s", ex.what());
    //       RCLCPP_WARN_THROTTLE(
    //           logger_, steady_clock, 1.0, "The tf from '%s' to '%s' is not available.",
    //           msg->header.frame_id.c_str(), target_frame_id_.c_str());
    //     }
    //     else
    //     {
    //       first_tf_error = false;
    //     }
    //   }
    //   catch (std::out_of_range &ex)
    //   {
    //     rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    //     RCLCPP_ERROR(logger_,
    //                  "Error transforming map from frame `%s` to frame `%s`: no layer with name `%s` is present",
    //                  msg->header.frame_id.c_str(),
    //                  target_frame_id_.c_str(),
    //                  ELEVATION_LAYER);
    //   }
    // }
  }
} // namespace zed_nav2

// Register the macro for this layer
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(zed_nav2::ZedCostmapLayer, nav2_costmap_2d::Layer)
