cmake_minimum_required(VERSION 3.8)
project(zed_ros2_wrapper_qa)

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
    set(ROS2_DISTROS "ardent;crystal;dashing;eloquent;foxy;galactic;humble;jazzy;rolling")
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
    if(${FOUND_ROS2_DISTRO} STREQUAL "foxy")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
        add_definitions(-DFOUND_FOXY)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "humble")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
        add_definitions(-DFOUND_HUMBLE)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "iron")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
        add_definitions(-DFOUND_IRON)
    else()
        message("*** WARNING *** ROS2 ${FOUND_ROS2_DISTRO} is not officially supported by the package '${PROJECT_NAME}'. Correct operation is not guaranteed.")
    endif()
else()
    message("*** WARNING *** ROS2 distro is unknown. This package could not work correctly.")
endif()
################################################

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE MATCHES Release)
    #message(" * Release Mode")
    add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    #message(" * Release with Debug Info Mode")
    add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
    message(" * Debug Mode")
endif()

#############################################
# Dependencies

set(DEPENDENCIES_COMP
    rclcpp
    rclcpp_components
    builtin_interfaces
    sensor_msgs
    zed_msgs
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)

find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(zed_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(yaml-cpp REQUIRED)

###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
file(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

###############################################################################
# INCLUDES and LIBS
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include
    ${CUDA_INCLUDE_DIRS}
)

link_directories(${CUDA_LIBRARY_DIRS})

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

###############################################################################
# SOURCES

set(INC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/ipc_visibility_control.hpp
)

set(PC_INC_COMPINC_COMP
    ${CMAKE_CURRENT_SOURCE_DIR}/src/components/include/performance_test_node.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/components/include/winavg.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/components/include/HardwareLoad.h
)

set(PC_SRC_COMP
    ${CMAKE_CURRENT_SOURCE_DIR}/src/components/src/performance_test_node.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/components/src/winavg.cpp
)

###############################################################################
# Bin and Install

# Component
add_library(performance_test_node_component SHARED
    ${INC}
    ${PC_INC_COMPINC_COMP}
    ${PC_SRC_COMP}
)
target_include_directories(performance_test_node_component PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/components/include
)
target_compile_definitions(performance_test_node_component
    PRIVATE "COMPOSITION_BUILDING_DLL"
)

target_link_libraries(performance_test_node_component
        yaml-cpp
        ${CUDA_CUDA_LIBRARY} 
        ${CUDA_CUDART_LIBRARY}
    )
ament_target_dependencies(performance_test_node_component
    ${DEPENDENCIES_COMP}
)

# Register component
rclcpp_components_register_nodes(performance_test_node_component "stereolabs::PerformanceTest")
set(node_plugins "${node_plugins}stereolabs::PerformanceTest;$<TARGET_FILE:performance_test_node_component>\n")

# Install components
install(TARGETS performance_test_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(performance_test_duration_node src/performance_test_duration_node.cpp)
ament_target_dependencies(performance_test_duration_node  rclcpp)

install(TARGETS
  performance_test_duration_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(
    DIRECTORY launch config scripts
    DESTINATION share/${PROJECT_NAME}/
    )

# Install header files
install(DIRECTORY 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/components/include/
    DESTINATION include/${PROJECT_NAME}/
)

# Install LAUNCH files
install(DIRECTORY
  launch launch config scripts results rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(
    pointcloud_component
)
ament_export_dependencies(
    ${DEPENDENCIES_COMP}
)
ament_package()
