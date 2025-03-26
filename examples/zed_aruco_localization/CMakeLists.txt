cmake_minimum_required(VERSION 3.8)
project(zed_aruco_localization)

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
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "jazzy")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
        add_definitions(-DFOUND_JAZZY)
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

find_package(OpenCV 4 REQUIRED)
find_package(ZED 4 REQUIRED)
find_package(CUDA REQUIRED)

set(DEPENDENCIES_COMP
    rclcpp
    rclcpp_components
    builtin_interfaces
    sensor_msgs
    image_transport
    zed_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(zed_components)

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
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include/aruco_loc_visibility_control.hpp
    )

    set(INC_COMP
        ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include/zed_aruco_localization_component.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include/aruco.hpp
    )

    set(SRC_COMP
        ${CMAKE_CURRENT_SOURCE_DIR}/src/component/src/zed_aruco_localization_component.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/component/src/aruco.cpp
    )

    ###############################################################################
    # Bin and Install

    # Component
    add_library(${PROJECT_NAME}_component SHARED
        ${INC}
        ${INC_COMP}
        ${SRC_COMP}
    )
    target_compile_definitions(${PROJECT_NAME}_component
        PRIVATE "COMPOSITION_BUILDING_DLL"
    )
    target_include_directories(${PROJECT_NAME}_component
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(${PROJECT_NAME}_component
        ${OpenCV_LIBS}
    )
    ament_target_dependencies(${PROJECT_NAME}_component
        ${DEPENDENCIES_COMP}
    )

    rclcpp_components_register_nodes(${PROJECT_NAME}_component "stereolabs::ZedArucoLoc")
    set(node_plugins "${node_plugins}stereolabs::ZedArucoLoc;$<TARGET_FILE:${PROJECT_NAME}_component>\n")

    # Install components
    install(TARGETS ${PROJECT_NAME}_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    )

    # Install header files
    install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include/
        ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include/
        DESTINATION include/${PROJECT_NAME}/
    )

    # Install LAUNCH files
    install(DIRECTORY
    launch config rviz2
    DESTINATION share/${PROJECT_NAME}
    )

    ament_export_include_directories(include)
    ament_export_libraries(${PROJECT_NAME}_component)
    ament_export_dependencies(${DEPENDENCIES_COMP})
endif()


ament_package()
