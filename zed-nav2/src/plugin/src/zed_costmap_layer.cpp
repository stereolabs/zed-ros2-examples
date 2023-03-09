#include <string>
#include <stdexcept>
#include <limits>
#include <sstream>

#include <nav2_costmap_2d/costmap_math.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <rclcpp/parameter_events_filter.hpp>
#include <tf2_eigen/tf2_eigen/tf2_eigen.hpp>
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
    min_traversability_cost_ = node->declare_parameter<float>(getFullName("min_traversability_cost"), min_traversability_cost_);

    // Get the path of the gridmap topic.
    std::string grid_map_topic = "/local_map/gridmap";

    grid_map_topic = node->declare_parameter<std::string>(
        getFullName("grid_map_topic"), grid_map_topic);

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

    RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Name: " << name_ << " - Min traversability cost: " << min_traversability_cost_);

    RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Name: " << name_ << " - Cost map resolution: " << getResolution());

    RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Name: " << name_ << " - Cost map size: X(" << getSizeInCellsX() << ") Y (" << getSizeInCellsY() << ")");

    // ----> TF2 Transform
    // mTfBuffer = std::make_unique<tf2_ros::Buffer>(clock_);
    // mTfListener = std::make_unique<tf2_ros::TransformListener>(*mTfBuffer); // Start TF Listener thread
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

    RCLCPP_DEBUG(logger_, "Updating bounds");
    bool layers_exist = checkLayersAndWarn();

    if (layered_costmap_->isRolling())
    {
      updateOrigin(
          robot_x - getSizeInMetersX() / 2,
          robot_y - getSizeInMetersY() / 2);
    }

    useExtraBounds(min_x, min_y, max_x, max_y);

    if (layers_exist)
    {
      // TODO (Patrick) update to match the grid map received instead of the entire costmap
      // Once this happens, fetching the information from the gridmap should be changed 
      // as to not ask for "out of bounds" value from the gridmap. It would throw an excpetion
      // it will loop back to its starting index, thus creating a repeating pattern
      double width = static_cast<int>(getSizeInCellsX()) * getResolution() / 2.0;
      double length = static_cast<int>(getSizeInCellsY()) * getResolution() / 2.0;
      *min_x = -width + robot_x;
      *max_x = width + robot_x;
      *min_y = -length  + robot_y;
      *max_y = length + robot_y;
    }
    else
    {
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
    RCLCPP_DEBUG(logger_, "Updating Costs");

    RCLCPP_DEBUG(
        node->get_logger(),
        "Update bounds: Min i: %d Min j: %d Max i: %d Max j: %d", min_i,
        min_j, max_i, max_j);

    // Size checks
    if (!checkLayersAndWarn())
    {
      return;
    }
    if (map_.getSize().prod() == 0)
    {
      RCLCPP_DEBUG(
          node->get_logger(),
          "Empty grid map when updating costs !");
      return;
    }
    // End of Size checks

    setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
    matchSize();
    uint8_t *costmap_array = master_grid.getCharMap();
    unsigned int size_x = getSizeInCellsX(), size_y = getSizeInCellsY();

    RCLCPP_DEBUG(
        node->get_logger(), "Costmap2D size in cells x: %d size in cells y: %d",
        size_x, size_y);

    unsigned int upper_bound = getIndex(max_i, max_j);
    unsigned int lower_bound = getIndex(min_i, min_j);
    unsigned int it = upper_bound;
    for (int j = min_j; j < max_j; j++)
    {
      for (int i = min_i; i < max_i; i++)
      {
        int index = getIndex(i, j);
        float traversability_value = std::numeric_limits<double>::quiet_NaN();
        try
        {
          // TODO (Patrick) update to safely fetch index without exceeding the bounds
          traversability_value = map_.at(TRAVERSABILITY_LAYER, grid_map::Index(i, j));
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
        // Calculate what this maps to in the original structure.
        uint8_t cost = nav2_costmap_2d::NO_INFORMATION;

        // ----> From ZED SDK
        constexpr float OCCUPIED_CELL = 1.f;
        constexpr float FREE_CELL = 0.f;
        constexpr float INVALID_CELL_DATA = NAN;
        constexpr float UNKNOWN_CELL = NAN;
        // <---- From ZED SDK

        // Convert the distance value to a costmap value.
        if (traversability_value == FREE_CELL || traversability_value <= min_traversability_cost_)
        {
          cost = nav2_costmap_2d::FREE_SPACE;
        }
        else if (traversability_value >= max_traversability_cost_ || traversability_value == OCCUPIED_CELL)
        {
          cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        }
        else if (isnan(traversability_value)) // traversability_value == INVALID_CELL_DATA || traversability_value == UNKNOWN_CELL
        {
          cost = nav2_costmap_2d::NO_INFORMATION;
        }
        else
        {
          cost = static_cast<uint8_t>(traversability_value * max_cost_value_);
        }
        // Reverse cell order because of different conventions between Costmap and grid map.
        int reversed_index = getIndex(max_i - i - 1, max_j - j - 1);
        if (reversed_index >= lower_bound && reversed_index <= upper_bound)
        {
          costmap_[reversed_index] = cost;
        }
        else
        {
          RCLCPP_DEBUG(logger_, "Out of bounds reversed index upper:%d lower:%d reversed index: %d.", i, j, reversed_index);
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

  bool ZedCostmapLayer::checkLayersAndWarn()
  {
    if (map_.exists(ELEVATION_LAYER) && map_.exists(RGB_LAYER) && map_.exists(TRAVERSABILITY_LAYER))
    {
      return true;
    }
    else
    {
      RCLCPP_WARN(
          logger_,
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
          logger_,
          "Gridmap not valid. Available layers: " << ss.str().c_str());

      return false;
    }

    return false;
  }

  void ZedCostmapLayer::gridmapCallback(
      const grid_map_msgs::msg::GridMap::ConstSharedPtr msg)
  {

    auto node = node_.lock();
    RCLCPP_DEBUG(node->get_logger(), "GridMap callback.");
    static bool first_tf_error = true;
    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }

    mGrid_mutex.lock();
    RCLCPP_DEBUG(node->get_logger(), "GridMap callback.");

    // Deserialize into grid map
    grid_map::GridMapRosConverter::fromMessage(*msg, map_);

    // Check the data matches the configuration
    auto size = map_.getSize();
    int grid_x = size.x();
    int grid_y = size.y();
    int cost_map_x = static_cast<int>(getSizeInCellsX());
    int cost_map_y = static_cast<int>(getSizeInCellsY());
    int res_grid = static_cast<int>(10000 * map_.getResolution());
    int res_cost = static_cast<int>(10000 * getResolution());

    if (grid_x != cost_map_x ||
        grid_y != cost_map_y ||
        res_grid != res_cost)
    {
      RCLCPP_WARN(logger_,
                  "The received grid map size (%d, %d) & resolution (%f) do not match the costmap size (%d, %d) and resolution (%f).",
                  size.x(), size.y(), map_.getResolution(),
                  getSizeInCellsX(), getSizeInCellsY(), getResolution());
    }

    if (grid_x == 0 || grid_y == 0)
    {
      RCLCPP_WARN(logger_, "Skipping empty  grid map");
      return;
    }

    // End of check

    target_frame_id_ = layered_costmap_->getGlobalFrameID();
    if (msg->header.frame_id != target_frame_id_)
    {
      RCLCPP_DEBUG(node->get_logger(), "Transforming map from %s to %s.", msg->header.frame_id.c_str(), target_frame_id_.c_str());
      try
      {
        auto res = tf_->canTransform(target_frame_id_,
                                           msg->header.frame_id,
                                           rclcpp::Time(0, 0, RCL_ROS_TIME),
                                           rclcpp::Duration::from_nanoseconds(1e4));

        if (!res)
        {
          RCLCPP_DEBUG(node->get_logger(), "Cannot transform from %s to %s.", msg->header.frame_id.c_str(), target_frame_id_.c_str());
          return;
        }

        auto tf_stamped = tf_->lookupTransform(target_frame_id_,
                                                     msg->header.frame_id,
                                                     rclcpp::Time(0, 0, RCL_ROS_TIME),
                                                     rclcpp::Duration::from_nanoseconds(1e4));
        {
          RCLCPP_DEBUG(logger_, "Applying transform");
          auto transform = tf2::transformToEigen(tf_stamped);
          map_ = map_.getTransformedMap(transform, ELEVATION_LAYER, target_frame_id_, 0.0);
          RCLCPP_DEBUG(logger_, "Transform applied");
        }
      }
      catch (tf2::TransformException &ex)
      {
        if (!first_tf_error)
        {
          rclcpp::Clock steady_clock(RCL_STEADY_TIME);
          RCLCPP_WARN_THROTTLE(logger_, steady_clock, 1.0, "Transform error: %s", ex.what());
          RCLCPP_WARN_THROTTLE(
              logger_, steady_clock, 1.0, "The tf from '%s' to '%s' is not available.",
              msg->header.frame_id.c_str(), target_frame_id_.c_str());
        }
        else
        {
          first_tf_error = false;
        }
      }
      catch (std::out_of_range &ex)
      {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_ERROR(logger_,
                     "Error transforming map from frame `%s` to frame `%s`: no layer with name `%s` is present",
                     msg->header.frame_id.c_str(),
                     target_frame_id_.c_str(),
                     TRAVERSABILITY_LAYER);
      }
    }

    mGrid_mutex.unlock();
  }
} // namespace zed_nav2

// Register the macro for this layer
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(zed_nav2::ZedCostmapLayer, nav2_costmap_2d::Layer)
