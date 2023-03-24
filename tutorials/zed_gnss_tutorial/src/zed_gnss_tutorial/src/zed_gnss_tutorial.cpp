#include "zed_gnss_tutorial/zed_gnss_tutorial.hpp"

// TODO: deal with qos
// TODO: Check transitions and sync
// TODO: Add ros2 wrapper, gps driver and rviz to launch file
// TODO: linter, comment and refactor
// TODO: Destructor, shutdown and error func

namespace zed_gnss_tutorial {

    ZedGnssTutorialNode::ZedGnssTutorialNode(const rclcpp::NodeOptions & options) : nav2_util::LifecycleNode("zed_gnss_tutorial_node","",options){
        
        RCLCPP_INFO(get_logger(),"****** ZED GNSS Tutorial Node *****");
        declare_params();
        
    }

    ZedGnssTutorialNode::~ZedGnssTutorialNode(){
    }

    nav2_util::CallbackReturn ZedGnssTutorialNode::on_configure(const rclcpp_lifecycle::State & /*state*/){

        RCLCPP_INFO(get_logger(),"***** Configuring Node *****");

        get_params();

        callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);    

        from_ll_to_map_client_ = create_client<robot_localization::srv::FromLL>(fromll_service_name_,rmw_qos_profile_services_default,callback_group_);
        to_ll_from_map_client_ = create_client<robot_localization::srv::ToLL>(toll_service_name_,rmw_qos_profile_services_default,callback_group_);

        if (!from_ll_to_map_client_->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_ERROR(get_logger(),"%s service is not ready.",fromll_service_name_.c_str());
            return nav2_util::CallbackReturn::ERROR;
        }

        if (!to_ll_from_map_client_->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_ERROR(get_logger(),"%s service is not ready.",toll_service_name_.c_str());
            return nav2_util::CallbackReturn::ERROR;
        }

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        thread_ = std::thread([this](){
            single_executor_.add_callback_group(callback_group_,get_node_base_interface());
            single_executor_.spin();
        });
        return nav2_util::CallbackReturn::SUCCESS;
    }
    
    nav2_util::CallbackReturn ZedGnssTutorialNode::on_activate(const rclcpp_lifecycle::State & /*state*/){

        RCLCPP_INFO(get_logger(),"***** Activating Node *****");

        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group_;
        mSubGeoPoseStamped = create_subscription<geographic_msgs::msg::GeoPoseStamped>(
            geopose_topic_name_,10,std::bind(&ZedGnssTutorialNode::geopose_callback,this,std::placeholders::_1),options);
        
        timer_=create_wall_timer(std::chrono::seconds(timer_period_sec_),std::bind(&ZedGnssTutorialNode::timer_callback,this),
            this->get_node_base_interface()->get_default_callback_group());

        createBond();
        return nav2_util::CallbackReturn::SUCCESS;
    }
    
    nav2_util::CallbackReturn ZedGnssTutorialNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/){
        
        RCLCPP_INFO(get_logger(),"***** Deactivating Node *****");

        timer_->cancel();
        if (!timer_->is_canceled()){
            RCLCPP_WARN(get_logger(),"Killing timer");
        }
        timer_.reset();
        mSubGeoPoseStamped.reset();

        destroyBond();
        return nav2_util::CallbackReturn::SUCCESS;
    }
    
    nav2_util::CallbackReturn ZedGnssTutorialNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/){

        RCLCPP_INFO(get_logger(),"***** Unconfiguring Node *****");

        tf_listener_.reset();
        tf_buffer_->clear();
        tf_buffer_.reset();

        to_ll_from_map_client_.reset();
        from_ll_to_map_client_.reset();
        
        single_executor_.cancel();
        if (thread_.joinable()){
            thread_.join();
        }
        callback_group_.reset();

        return nav2_util::CallbackReturn::SUCCESS;
    }
    
    nav2_util::CallbackReturn ZedGnssTutorialNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/){

        RCLCPP_INFO(get_logger(),"***** Shutting down Node *****");

        return nav2_util::CallbackReturn::SUCCESS;
    }

    void ZedGnssTutorialNode::declare_params(){

        declare_parameter("target_frame_name",target_frame_name_);
        declare_parameter("source_frame_name",source_frame_name_);
        declare_parameter("geopose_topic_name",geopose_topic_name_);
        declare_parameter("fromll_service_name",fromll_service_name_);
        declare_parameter("toll_service_name",toll_service_name_);
        declare_parameter("timer_period_sec",timer_period_sec_);
    }

    void ZedGnssTutorialNode::get_params(){

        get_parameter("target_frame_name",target_frame_name_);
        get_parameter("source_frame_name",source_frame_name_);
        get_parameter("geopose_topic_name",geopose_topic_name_);
        get_parameter("fromll_service_name",fromll_service_name_);
        get_parameter("toll_service_name",toll_service_name_);
        get_parameter("timer_period_sec",timer_period_sec_);

    }

    void ZedGnssTutorialNode::geopose_callback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg){
        
        latest_geopose_msg_ = *msg.get();
    }

    void ZedGnssTutorialNode::timer_callback(){

        RCLCPP_INFO(get_logger(),"***** Timer Callback *****");

        if (mSubGeoPoseStamped->get_publisher_count()==0){
            RCLCPP_ERROR(get_logger(),"No publisher for %s has been registered",geopose_topic_name_.c_str());
            return;
        }
        RCLCPP_INFO(
            get_logger(), "%s GPS coordinates: Altitude: %.2f Latitude: %.2f Longitude: %.2f",target_frame_name_.c_str(),
            latest_geopose_msg_.pose.position.altitude,latest_geopose_msg_.pose.position.latitude,latest_geopose_msg_.pose.position.longitude);

        geometry_msgs::msg::TransformStamped pose_wrt_map;
        try{
            pose_wrt_map = tf_buffer_->lookupTransform(
            target_frame_name_,source_frame_name_,tf2::TimePointZero);
            RCLCPP_INFO(
            get_logger(),"%s coordinates in %s: X: %.2f Y: %.2f Z: %.2f",target_frame_name_.c_str(),source_frame_name_.c_str(),
            pose_wrt_map.transform.translation.x, pose_wrt_map.transform.translation.y, pose_wrt_map.transform.translation.z);
        } catch (const tf2::TransformException &ex){
            RCLCPP_ERROR(get_logger(),"No transformation available for %s from %s",target_frame_name_.c_str(),source_frame_name_.c_str());
            return;
        }
   
        auto from_ll_to_map_request = std::make_shared<robot_localization::srv::FromLL::Request>();
        from_ll_to_map_request->ll_point.altitude=latest_geopose_msg_.pose.position.altitude;
        from_ll_to_map_request->ll_point.latitude=latest_geopose_msg_.pose.position.latitude;
        from_ll_to_map_request->ll_point.longitude=latest_geopose_msg_.pose.position.longitude;
        
        auto from_ll_to_map_response = from_ll_to_map_client_->async_send_request(from_ll_to_map_request);
        auto future_status = from_ll_to_map_response.wait_for(std::chrono::milliseconds(200));
        if (future_status==std::future_status::ready){
            auto from_ll_to_map_response_data = from_ll_to_map_response.get();
            RCLCPP_INFO(
                get_logger(),"%s GPS coordinates converted back to %s:  X: %.2f Y: %.2f Z: %.2f",
                target_frame_name_.c_str(),source_frame_name_.c_str(),from_ll_to_map_response_data->map_point.x,
                from_ll_to_map_response_data->map_point.y,from_ll_to_map_response_data->map_point.z);
        }
        else{
            RCLCPP_WARN(get_logger(),"%s service took more than 200ms to answer. Skipping...",fromll_service_name_.c_str());
        }
        
        auto to_ll_from_map_request = std::make_shared<robot_localization::srv::ToLL::Request>();
        to_ll_from_map_request->map_point.x=pose_wrt_map.transform.translation.x;
        to_ll_from_map_request->map_point.y=pose_wrt_map.transform.translation.y;
        to_ll_from_map_request->map_point.z=pose_wrt_map.transform.translation.z;
        auto to_ll_from_map_response = to_ll_from_map_client_->async_send_request(to_ll_from_map_request);
        future_status = to_ll_from_map_response.wait_for(std::chrono::milliseconds(200));
        if (future_status==std::future_status::ready){
            auto to_ll_from_map_response_data = to_ll_from_map_response.get();
            RCLCPP_INFO(
                get_logger(),"%s coordinates in %s converted back to GPS: Altitude: %.2f Latitude: %.2f Longitude: %.2f",
                target_frame_name_.c_str(),source_frame_name_.c_str(),to_ll_from_map_response_data->ll_point.altitude,
                to_ll_from_map_response_data->ll_point.latitude, to_ll_from_map_response_data->ll_point.longitude);
        }
        else{
            RCLCPP_WARN(get_logger(),"%s service took more than 200ms to answer. Skipping...",toll_service_name_.c_str());
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(zed_gnss_tutorial::ZedGnssTutorialNode)