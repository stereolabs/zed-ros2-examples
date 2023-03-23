#ifndef ZED_GNSS_TUTORIAL_HPP
#define ZED_GNSS_TUTORIAL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <robot_localization/srv/from_ll.hpp>
#include <robot_localization/srv/to_ll.hpp>

namespace zed_gnss_tutorial{
    class ZedGnssTutorialNode : public nav2_util::LifecycleNode{
        
        public:
        
        ZedGnssTutorialNode(const rclcpp::NodeOptions & options);
        ~ZedGnssTutorialNode() override;

        protected:
        
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/) override;
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/) override;
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/) override;
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*state*/) override;
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*state*/) override;

        private:
        
        std::shared_ptr<rclcpp::CallbackGroup> callback_group_;
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> single_executor_;
        std::unique_ptr<std::thread> thread_;

        std::shared_ptr<rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>> mSubGeoPoseStamped;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::shared_ptr<rclcpp::Client<robot_localization::srv::FromLL>> from_ll_to_map_client_;
        std::shared_ptr<rclcpp::Client<robot_localization::srv::ToLL>> fmap_to_ll_client_;
    };
}
#endif