cmake_minimum_required(VERSION 3.8)
project(zed_topic_benchmark_interfaces)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################################################
# Check the ROS2 version

set(ROS2_FOUND FALSE)
if(DEFINED ENV{ROS_DISTRO})
    set(FOUND_ROS2_DISTRO $ENV{ROS_DISTRO})
    set(ROS2_FOUND TRUE)
    #message("* Found ROS2 ${FOUND_ROS2_DISTRO}")
else()
    message("* ROS2 distro variable not set. Trying to figure it out...")
    set(ROS2_DISTROS "ardent;bouncy;crystal;dashing;eloquent;foxy;galactic;humble;iron;jazzy;kilted;lyrical;rolling")
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
endif()

if(ROS2_FOUND)
    if(${FOUND_ROS2_DISTRO} STREQUAL "humble")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
        add_definitions(-DFOUND_HUMBLE)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "jazzy")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
        add_definitions(-DFOUND_JAZZY)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "lyrical")
        add_definitions(-DFOUND_LYRICAL)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "rolling")
        add_definitions(-DFOUND_ROLLING)
    else()
        message("*** WARNING *** Unsupported ROS2 ${FOUND_ROS2_DISTRO}. '${PROJECT_NAME}' may not work correctly.")
    endif()
else()
    message("*** WARNING *** ROS2 distro is unknown. This package could not work correctly.")
endif()
################################################

################################################
# The benchmark tool subscribes to topics whose type is only known at runtime
# ("generic subscriptions"). That API is missing in Foxy and older
# distributions, so on those the package is still registered but nothing is
# built. Every newer distribution (Humble, Jazzy, Lyrical, Rolling, ...) is
# built: using an exclusion list instead of an allow list means a new ROS 2
# release does not silently produce an empty package.
set(ZED_BENCHMARK_UNSUPPORTED_DISTROS "ardent;bouncy;crystal;dashing;eloquent;foxy")
if("${FOUND_ROS2_DISTRO}" IN_LIST ZED_BENCHMARK_UNSUPPORTED_DISTROS)
    message("*** WARNING *** ROS2 ${FOUND_ROS2_DISTRO} lacks generic topic subscriptions: '${PROJECT_NAME}' will not be built.")
    set(ZED_BENCHMARK_BUILD FALSE)
else()
    set(ZED_BENCHMARK_BUILD TRUE)
endif()
################################################

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

###############################################################################
# Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
file(GLOB_RECURSE extra_files ${CMAKE_CURRENT_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${extra_files})
###############################################################################

if(ZED_BENCHMARK_BUILD)
    set(MSG_FILES
        "msg/BenchmarkStatsStamped.msg"
    )

    set(SRV_FILES
    )

    rosidl_generate_interfaces(${PROJECT_NAME}
        ${MSG_FILES}
        ${SRV_FILES}
        DEPENDENCIES builtin_interfaces std_msgs
    )

    ament_export_dependencies(rosidl_default_runtime)
endif()

ament_package()
