cmake_minimum_required(VERSION 3.8)
project(zed_gnss_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_localization REQUIRED)

include_directories(include)

add_library(zed_gnss_tutorial_lib SHARED src/zed_gnss_tutorial.cpp)
target_include_directories(zed_gnss_tutorial_lib PRIVATE src/include)
target_include_directories(zed_gnss_tutorial_lib PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(zed_gnss_tutorial_lib)

ament_target_dependencies(zed_gnss_tutorial_lib
    rclcpp
    rclcpp_lifecycle
    nav2_util
    lifecycle_msgs
    rclcpp_components
    nav2_lifecycle_manager
    geographic_msgs
    tf2
    tf2_ros
    robot_localization
)

rclcpp_components_register_nodes(zed_gnss_tutorial_lib "zed_gnss_tutorial::ZedGnssTutorialNode")

install (TARGETS zed_gnss_tutorial_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY
    config
    include
    launch
    rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(zed_gnss_tutorial_lib)
ament_package()