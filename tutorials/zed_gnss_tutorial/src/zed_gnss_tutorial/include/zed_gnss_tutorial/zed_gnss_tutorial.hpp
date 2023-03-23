#ifndef ZED_GNSS_TUTORIAL_HPP
#define ZED_GNSS_TUTORIAL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

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
        
        int foo;

    };
}
#endif