cmake_minimum_required(VERSION 3.8)
project(zed_topic_benchmark)

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
    set(ROS2_DISTROS "ardent;crystal;dashing;eloquent;foxy;galactic;humble;jazzy;lyrical;rolling")
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
  if("${FOUND_ROS2_DISTRO}" STREQUAL "humble")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
        add_definitions(-DFOUND_HUMBLE)
  elseif("${FOUND_ROS2_DISTRO}" STREQUAL "jazzy")
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

if("${FOUND_ROS2_DISTRO}" STREQUAL "humble" OR 
   "${FOUND_ROS2_DISTRO}" STREQUAL "iron" OR 
   "${FOUND_ROS2_DISTRO}" STREQUAL "jazzy")
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
    zed_topic_benchmark_component
  )

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(zed_topic_benchmark_component REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    # launch_testing integration test: runs the benchmark node against a
    # synthetic publisher and checks the report/termination behavior.
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(
      test/test_benchmark_integration.py
      TIMEOUT 120
    )
  endif()

  ###############################################################################
  # Add all files in subdirectories of the project in
  # a dummy_target so qtcreator have access to all files
  file(GLOB_RECURSE extra_files ${CMAKE_CURRENT_SOURCE_DIR}/*)
  add_custom_target(${PROJECT_NAME}_files SOURCES ${extra_files})
  ###############################################################################

  ###############################################################################
  # ament_target_dependencies() replacement: ROS 2 Lyrical's ament_cmake
  # removed that macro outright. For each dependency, link its exported
  # modern CMake target(s) (${dep}_TARGETS), falling back to the classic
  # ${dep}_INCLUDE_DIRS / ${dep}_LIBRARIES variables for dependencies that
  # only export those.
  macro(zed_link_ros_dependencies target)
    foreach(_dep ${ARGN})
      if(${_dep}_TARGETS)
        target_link_libraries(${target} ${${_dep}_TARGETS})
      else()
        if(${_dep}_INCLUDE_DIRS)
          target_include_directories(${target} PUBLIC ${${_dep}_INCLUDE_DIRS})
        endif()
        if(${_dep}_LIBRARIES)
          target_link_libraries(${target} ${${_dep}_LIBRARIES})
        endif()
      endif()
    endforeach()
  endmacro()

  ## Sources
  set(SRC_CONT src/topic_benchmark_container.cpp)

  ## Build
  add_executable(${PROJECT_NAME} ${SRC_CONT})
  zed_link_ros_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

  # Install executable
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  # Install LAUNCH files
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )

  # Install helper scripts (runnable via `ros2 run ${PROJECT_NAME} <script>`)
  install(PROGRAMS
    scripts/zed_check_ros2_config.sh
    DESTINATION lib/${PROJECT_NAME}
  )

  ament_package()
endif()


