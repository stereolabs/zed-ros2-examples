#include "zed_gnss_tutorial/zed_gnss_tutorial.hpp"

namespace zed_gnss_tutorial {

    ZedGnssTutorialNode::ZedGnssTutorialNode(const rclcpp::NodeOptions & options) : nav2_util::LifecycleNode("zed_gnss_tutorial_node","",options){

    }

    ZedGnssTutorialNode::~ZedGnssTutorialNode(){

    }

    nav2_util::CallbackReturn ZedGnssTutorialNode::on_configure(const rclcpp_lifecycle::State & /*state*/){
        return nav2_util::CallbackReturn::SUCCESS;
    }
    
    nav2_util::CallbackReturn ZedGnssTutorialNode::on_activate(const rclcpp_lifecycle::State & /*state*/){
        return nav2_util::CallbackReturn::SUCCESS;
    }
    
    nav2_util::CallbackReturn ZedGnssTutorialNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/){
        return nav2_util::CallbackReturn::SUCCESS;
    }
    
    nav2_util::CallbackReturn ZedGnssTutorialNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/){
        return nav2_util::CallbackReturn::SUCCESS;
    }
    
    nav2_util::CallbackReturn ZedGnssTutorialNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/){
        return nav2_util::CallbackReturn::SUCCESS;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(zed_gnss_tutorial::ZedGnssTutorialNode)