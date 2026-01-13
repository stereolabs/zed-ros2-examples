cmake_minimum_required(VERSION 3.8)
project(zed_benchmark_components)

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
        message("*** WARNING *** Unsupported ROS2 ${FOUND_ROS2_DISTRO}. '${PROJECT_NAME}' may not work correctly.")
    endif()
else()
    message("*** WARNING *** ROS2 distro is unknown. This package could not work correctly.")
endif()
################################################

if(${FOUND_ROS2_DISTRO} STREQUAL "humble" OR ${FOUND_ROS2_DISTRO} STREQUAL "iron")
  # if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
  if(NOT CMAKE_BUILD_TYPE)
      set(CMAKE_BUILD_TYPE Release)
  endif()

  # Default to C++17
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()

  # dependencies
  set(DEPENDENCIES
    rclcpp
    rclcpp_components    
    zed_topic_benchmark_interfaces
  )

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()
  
  find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)

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
  
  ## INCLUDES and LIBS
  include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${CUDA_INCLUDE_DIRS}
  ) 

  link_directories(${CUDA_LIBRARY_DIRS})

  ## SOURCES
  
  set(TOPIC_BENCHMARK_SRC_COMP
    src/zed_topic_benchmark/src/topic_benchmark_component.cpp
    src/zed_topic_benchmark/src/winavg.cpp)
    
  ## Topic Benchmark Component
  
  add_library(zed_topic_benchmark_component SHARED
    ${TOPIC_BENCHMARK_SRC_COMP}
  )
  target_compile_definitions(zed_topic_benchmark_component
    PRIVATE "COMPOSITION_BUILDING_DLL"
  )
  ament_target_dependencies(zed_topic_benchmark_component
    ${DEPENDENCIES}
  )

  target_include_directories(zed_topic_benchmark_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/zed_topic_benchmark/include>
    $<INSTALL_INTERFACE:include>
  )

  rclcpp_components_register_nodes(zed_topic_benchmark_component "stereolabs::TopicBenchmarkComponent")
  set(node_plugins "${node_plugins}stereolabs::TopicBenchmarkComponent;$<TARGET_FILE:zed_topic_benchmark_component>\n")
  

  install(TARGETS zed_topic_benchmark_component 
  EXPORT zed_benchmark_componentsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

  install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_topic_benchmark/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/
    DESTINATION include/${PROJECT_NAME}
  )

  ament_export_include_directories(include)
  ament_export_targets(zed_benchmark_componentsTargets HAS_LIBRARY_TARGET)
  ament_export_dependencies(${DEPENDENCIES})
  ament_package()
endif()




