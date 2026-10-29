cmake_minimum_required(VERSION 3.8)
project(zed_isaac_ros_cuda_img_sub)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#############################################
# Dependencies

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# The CUDA zero-copy transport requires `rosidl::Buffer` (ROS 2 Lyrical and
# newer) and the `cuda_buffer` package. The example is not built without them.
find_package(cuda_buffer QUIET)

if(cuda_buffer_FOUND)
  # Used to check that the 'cuda' buffer backend plugin is installed
  find_package(rosidl_buffer_backend_registry REQUIRED)

  set(DEPENDENCIES_COMP
    rclcpp
    rclcpp_components
    rcutils
    sensor_msgs
    cuda_buffer
    rosidl_buffer_backend_registry
  )

  find_package(rcutils REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(CUDAToolkit REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ###############################################################################
  # Add all files in subdirectories of the project in
  # a dummy_target so qtcreator have access to all files
  file(GLOB_RECURSE all_files ${CMAKE_CURRENT_SOURCE_DIR}/*)
  add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

  ###############################################################################
  # INCLUDES

  include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include
  )

  ###############################################################################
  # SOURCES

  set(INC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/cuda_img_sub_visibility_control.hpp
  )

  set(INC_COMP
    ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include/zed_cuda_img_sub_component.hpp
  )

  set(SRC_COMP
    ${CMAKE_CURRENT_SOURCE_DIR}/src/component/src/zed_cuda_img_sub_component.cpp
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
    PRIVATE "ZED_CUDA_IMG_SUB_COMPONENT_BUILDING_DLL"
  )
  target_link_libraries(${PROJECT_NAME}_component
    rclcpp::rclcpp
    rclcpp_components::component
    ${sensor_msgs_TARGETS}
    cuda_buffer::cuda_buffer
    rosidl_buffer_backend_registry::rosidl_buffer_backend_registry
    CUDA::cudart
    ${CMAKE_DL_LIBS}
  )

  rclcpp_components_register_nodes(${PROJECT_NAME}_component "stereolabs::ZedCudaImgSubComponent")

  # Install components
  install(TARGETS ${PROJECT_NAME}_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  # Install header files
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/include/
    DESTINATION include/${PROJECT_NAME}/
  )

  # Install LAUNCH and CONFIG files
  install(DIRECTORY
    launch config
    DESTINATION share/${PROJECT_NAME}
  )
else()
  message(WARNING
    " * 'cuda_buffer' not found: '${PROJECT_NAME}' requires ROS 2 Lyrical or newer "
    "with the 'cuda_buffer' package and will not be built.")
endif()

ament_package()
