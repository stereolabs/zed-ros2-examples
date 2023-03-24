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

        void geopose_callback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg);
        void timer_callback();
        void declare_params();
        void get_params();
        
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor single_executor_;
        std::thread thread_;

        rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr mSubGeoPoseStamped;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr from_ll_to_map_client_; 
        rclcpp::Client<robot_localization::srv::ToLL>::SharedPtr to_ll_from_map_client_;

        rclcpp::TimerBase::SharedPtr timer_;

        geographic_msgs::msg::GeoPoseStamped latest_geopose_msg_;

        std::string target_frame_name_{"base_link"};
        std::string source_frame_name_{"map"};
        std::string geopose_topic_name_{"geo_pose"};
        std::string fromll_service_name_{"fromLL"};
        std::string toll_service_name_{"toLL"};
        int timer_period_sec_{5};
    };
}
#endif