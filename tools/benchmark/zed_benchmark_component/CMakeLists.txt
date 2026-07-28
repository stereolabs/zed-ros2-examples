cmake_minimum_required(VERSION 3.8)
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

find_package(ament_cmake REQUIRED)

if(ZED_BENCHMARK_BUILD)
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
    sensor_msgs
    zed_topic_benchmark_interfaces
  )

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(zed_topic_benchmark_interfaces REQUIRED)

  ###############################################################################
  # Optional transport message types.
  #
  # image_transport and point_cloud_transport publish a topic once per transport
  # plugin, each with its own message type. Those plugins are separate,
  # individually installable packages, so each type is compiled in only when its
  # package is available - the benchmark must not require them all. Missing ones
  # simply fall back to the generic subscription.
  #
  # sensor_msgs/CompressedImage covers the compressed, compressedDepth and zstd
  # image transports, and needs no extra package.
  set(ZED_BENCHMARK_TRANSPORT_TYPES "")
  foreach(_pkg_def
      "theora_image_transport;ZED_BENCHMARK_HAS_THEORA"
      "ffmpeg_image_transport_msgs;ZED_BENCHMARK_HAS_FFMPEG"
      "point_cloud_interfaces;ZED_BENCHMARK_HAS_PC_INTERFACES")
    list(GET _pkg_def 0 _pkg)
    list(GET _pkg_def 1 _define)
    find_package(${_pkg} QUIET)
    if(${_pkg}_FOUND)
      list(APPEND DEPENDENCIES ${_pkg})
      list(APPEND ZED_BENCHMARK_TRANSPORT_DEFINES ${_define})
      list(APPEND ZED_BENCHMARK_TRANSPORT_TYPES ${_pkg})
    endif()
  endforeach()
  if(ZED_BENCHMARK_TRANSPORT_TYPES)
    message(STATUS
      "* ${PROJECT_NAME}: extra transport types ENABLED for "
      "${ZED_BENCHMARK_TRANSPORT_TYPES}")
  else()
    message(STATUS
      "* ${PROJECT_NAME}: no extra transport packages found; only the "
      "sensor_msgs transports are typed")
  endif()
  ###############################################################################

  ###############################################################################
  # Optional ZED type-adapted (zero-copy) subscription path.
  #
  # Only a typed subscription can take the intra-process path, and only one
  # built on the *same* TypeAdapter as the publisher receives the publisher's
  # buffer by pointer instead of a converted copy. The ZED wrapper publishes
  # images through TypeAdapter<StampedSlMat, sensor_msgs::msg::Image>, so this
  # path needs zed_components (for the adapter) and the ZED SDK (for sl::Mat).
  #
  # The dependency is deliberately optional: without it the benchmark still
  # builds and still measures the plain typed intra-process path, it just cannot
  # report true zero-copy.
  find_package(zed_components QUIET)
  find_package(CUDAToolkit QUIET)
  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.27")
    cmake_policy(SET CMP0146 OLD)
  endif()
  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.28")
    cmake_policy(SET CMP0153 OLD)
  endif()
  find_package(ZED QUIET)
  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.27")
    cmake_policy(SET CMP0146 NEW)
  endif()
  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.28")
    cmake_policy(SET CMP0153 NEW)
  endif()

  if(zed_components_FOUND AND ZED_FOUND AND CUDAToolkit_FOUND)
    set(ZED_BENCHMARK_ZERO_COPY TRUE)
    message(STATUS "* ${PROJECT_NAME}: ZED type-adapted zero-copy path ENABLED")
  else()
    set(ZED_BENCHMARK_ZERO_COPY FALSE)
    message(STATUS
      "* ${PROJECT_NAME}: ZED type-adapted zero-copy path DISABLED "
      "(needs zed_components + ZED SDK + CUDA). The typed intra-process path "
      "is still available.")
  endif()
  ###############################################################################


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
  set(SRC_COMP
    src/component/src/topic_benchmark_component.cpp
    src/component/src/typed_subscriber.cpp
    src/component/src/cpu_meter.cpp
    src/component/src/winavg.cpp)

  add_library(${PROJECT_NAME} SHARED
    ${SRC_COMP}
  )
  target_compile_definitions(${PROJECT_NAME}
    PRIVATE "COMPOSITION_BUILDING_DLL"
  )
  zed_link_ros_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

  if(ZED_BENCHMARK_TRANSPORT_DEFINES)
    target_compile_definitions(${PROJECT_NAME}
      PRIVATE ${ZED_BENCHMARK_TRANSPORT_DEFINES}
    )
  endif()

  if(ZED_BENCHMARK_ZERO_COPY)
    target_compile_definitions(${PROJECT_NAME}
      PRIVATE "ZED_BENCHMARK_HAS_SL_ADAPTER"
    )
    target_include_directories(${PROJECT_NAME} PRIVATE
      ${zed_components_INCLUDE_DIRS}
      ${ZED_INCLUDE_DIRS}
      ${CUDAToolkit_INCLUDE_DIRS}
    )
    target_link_directories(${PROJECT_NAME} PRIVATE ${ZED_LIBRARY_DIR})
    # NOTE: the ZED SDK's own CMake config exports ZED_LIBRARIES. ZED_LIBS is a
    # variable local to the zed_components package and is not available here.
    target_link_libraries(${PROJECT_NAME} ${ZED_LIBRARIES} CUDA::cudart)
  endif()

  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/component/include>
    $<INSTALL_INTERFACE:include>
  )

  ###############################################################################
  # Tests. Placed after the library so they can link it and use the
  # zed_link_ros_dependencies() macro defined above.
  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    # Unit tests for the statistics engine (no ROS dependencies: the WinAvg
    # source is compiled directly into each test executable).
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_winavg
      test/test_winavg.cpp
      src/component/src/winavg.cpp)
    target_include_directories(test_winavg PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include)

    ament_add_gtest(test_statistics
      test/test_statistics.cpp
      src/component/src/winavg.cpp)
    target_include_directories(test_statistics PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include)

    # CPU accounting: no ROS dependency, the source is compiled directly in.
    ament_add_gtest(test_cpu_meter
      test/test_cpu_meter.cpp
      src/component/src/cpu_meter.cpp)
    target_include_directories(test_cpu_meter PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include)

    # QoS parameter mapping. Pure logic, but it lives on the component class,
    # so the test links the library.
    ament_add_gtest(test_qos test/test_qos.cpp)
    target_include_directories(test_qos PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include
      ${CMAKE_CURRENT_SOURCE_DIR}/src/include)
    zed_link_ros_dependencies(test_qos ${DEPENDENCIES})
    target_link_libraries(test_qos ${PROJECT_NAME})

    # Typed/zero-copy subscription paths. These need a real ROS context, so the
    # test links the component library rather than recompiling the sources.
    ament_add_gtest(test_typed_subscriber test/test_typed_subscriber.cpp)
    target_include_directories(test_typed_subscriber PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include)
    zed_link_ros_dependencies(test_typed_subscriber ${DEPENDENCIES})
    target_link_libraries(test_typed_subscriber ${PROJECT_NAME})
    if(ZED_BENCHMARK_TRANSPORT_DEFINES)
      target_compile_definitions(test_typed_subscriber
        PRIVATE ${ZED_BENCHMARK_TRANSPORT_DEFINES})
    endif()
    if(ZED_BENCHMARK_ZERO_COPY)
      target_compile_definitions(test_typed_subscriber
        PRIVATE "ZED_BENCHMARK_HAS_SL_ADAPTER")
      target_include_directories(test_typed_subscriber PRIVATE
        ${zed_components_INCLUDE_DIRS}
        ${ZED_INCLUDE_DIRS}
        ${CUDAToolkit_INCLUDE_DIRS})
      target_link_directories(test_typed_subscriber PRIVATE ${ZED_LIBRARY_DIR})
      target_link_libraries(test_typed_subscriber ${ZED_LIBRARIES} CUDA::cudart)
    endif()
  endif()

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
endif()

# Always call ament_package(), even when nothing is built, so that the package
# is correctly registered in the ament index on unsupported distributions
# instead of failing the workspace build.
ament_package()




