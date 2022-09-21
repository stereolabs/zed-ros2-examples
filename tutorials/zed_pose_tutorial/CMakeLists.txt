cmake_minimum_required(VERSION 3.10)

project(zed_tutorial_pos_tracking)

################################################
# Check the ROS2 version

set(ROS2_FOUND FALSE)
if(DEFINED ENV{ROS_DISTRO})
    set(FOUND_ROS2_DISTRO $ENV{ROS_DISTRO})
    set(ROS2_FOUND TRUE)
    message("* Found ROS2 ${FOUND_ROS2_DISTRO}")
else()
    message("* ROS2 distro variable not set. Trying to figure it out...")
    set(ROS2_DISTROS "ardent;crystal;dashing;eloquent;foxy;galactic;humble;rolling")
    set(ROS2_FOUND FALSE)
    foreach(distro ${ROS2_DISTROS})
        if(NOT ROS2_FOUND)
            find_path(RCLCPP_H rclcpp.hpp PATHS /opt/ros/${distro}/include/rclcpp)
            if(RCLCPP_H)
                message("* Found ROS2 ${distro}")
                set(FOUND_ROS2_DISTRO ${distro})
                set(ROS2_FOUND TRUE)
            endif()
        endif()        
    endforeach()    
endif(DEFINED ENV{ROS_DISTRO})

if(ROS2_FOUND)
    add_compile_definitions(FOUND_ROS2_DISTRO=${FOUND_ROS2_DISTRO})
    if(${FOUND_ROS2_DISTRO} STREQUAL "foxy")
        message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "humble")
        message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
    else()
        message("*** WARNING *** ROS2 ${FOUND_ROS2_DISTRO} is not officially supported by this package. Correct working is not guarantee.")
    endif()
else()
    message("*** WARNING *** ROS2 distro is unknown. This package could not work correctly.")
endif()
################################################

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

###############################################################################
# Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(${PROJECT_NAME}_files SOURCES ${extra_files})
###############################################################################

# includes
include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)

## Build
add_executable(zed_tutorial_pos_tracking src/zed_tutorial_pos_tracking.cpp)
ament_target_dependencies(zed_tutorial_pos_tracking
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Install executable
install(
    TARGETS zed_tutorial_pos_tracking
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
