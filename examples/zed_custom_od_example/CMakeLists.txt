cmake_minimum_required(VERSION 3.5)
project(zed_custom_od_example)

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
    if(${FOUND_ROS2_DISTRO} STREQUAL "humble")
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

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
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
    vision_msgs
    image_transport
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED)


if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
endif()

###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
file(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

if( ${FOUND_ROS2_DISTRO} STREQUAL "humble" OR ${FOUND_ROS2_DISTRO} STREQUAL "iron")
    ###############################################################################
    # SOURCES

    set(INC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include/custom_od_visibility_control.hpp
    )

    set(INC_COMP
        ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include/custom_od_component_base.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include/yolo_detector.hpp
    )

    set(SRC_COMP
        ${CMAKE_CURRENT_SOURCE_DIR}/src/component/src/custom_od_component_base.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/component/src/yolo_detector.cpp
    )

    ###############################################################################
    # Bin and Install

    # Component
    add_library(zed_yolo_detector_component SHARED
        ${INC}
        ${INC_COMP}
        ${SRC_COMP}
    )
    target_compile_definitions(zed_yolo_detector_component
        PRIVATE "COMPOSITION_BUILDING_DLL"
    )
    target_include_directories(zed_yolo_detector_component
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(zed_yolo_detector_component
        ${OpenCV_LIBS}
    )
    ament_target_dependencies(zed_yolo_detector_component
        ${DEPENDENCIES_COMP}
    )

    rclcpp_components_register_nodes(zed_yolo_detector_component "stereolabs::ZedYoloDetector")
    set(node_plugins "${node_plugins}stereolabs::ZedYoloDetector;$<TARGET_FILE:zed_yolo_detector_component>\n")

    # Install components
    install(TARGETS zed_yolo_detector_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/zed_yolo_detector
    )

    # Install header files
    install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/include/
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include/
        DESTINATION include/zed_yolo_detector/
    )

    # Install LAUNCH files
    install(DIRECTORY
    launch config
    DESTINATION share/zed_yolo_detector
    )

    ament_export_include_directories(include)
    ament_export_libraries(zed_yolo_detector_component)
    ament_export_dependencies(${DEPENDENCIES_COMP})
endif()


ament_package()
