cmake_minimum_required(VERSION 3.10)
project(zed_topic_benchmark_component)

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
endif()

if(ROS2_FOUND)
    if(${FOUND_ROS2_DISTRO} STREQUAL "humble")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
        add_definitions(-DFOUND_HUMBLE)
    else()
        message("*** WARNING *** ROS2 ${FOUND_ROS2_DISTRO} is not supported. This package will not be installed.")
    endif()
else()
    message("*** WARNING *** ROS2 distro is unknown. This package will not be installed.")
endif()
################################################

if(${FOUND_ROS2_DISTRO} STREQUAL "humble")
  # if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
  if(NOT CMAKE_BUILD_TYPE)
      set(CMAKE_BUILD_TYPE Release)
  endif()

  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  # dependencies
  set(DEPENDENCIES
    rclcpp
    rclcpp_components
    zed_topic_benchmark_interfaces
  )

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(zed_topic_benchmark_interfaces REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ###############################################################################
  # Add all files in subdirectories of the project in
  # a dummy_target so qtcreator have access to all files
  file(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
  add_custom_target(${PROJECT_NAME}_files SOURCES ${extra_files})
  ###############################################################################

  ## Sources
  set(SRC_COMP
    src/component/src/topic_benchmark_component.cpp
    src/component/src/winavg.cpp)

  add_library(${PROJECT_NAME} SHARED
    ${SRC_COMP}
  )
  target_compile_definitions(${PROJECT_NAME}
    PRIVATE "COMPOSITION_BUILDING_DLL"
  )
  ament_target_dependencies(${PROJECT_NAME}
    ${DEPENDENCIES}
  )

  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/component/include>
    $<INSTALL_INTERFACE:include>
  )

  rclcpp_components_register_nodes(${PROJECT_NAME} "stereolabs::TopicBenchmarkComponent")
  set(node_plugins "${node_plugins}stereolabs::TopicBenchmarkComponent;$<TARGET_FILE:${PROJECT_NAME}>\n")

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/
    DESTINATION include/${PROJECT_NAME}
  )

  ament_export_include_directories(include)
  ament_export_libraries(${PROJECT_NAME})
  ament_export_dependencies(${DEPENDENCIES})
  ament_package()

endif()


