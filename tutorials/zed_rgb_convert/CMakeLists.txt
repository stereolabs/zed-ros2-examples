cmake_minimum_required(VERSION 3.5)
project(zed_rgb_convert)

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
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

if(CMAKE_BUILD_TYPE MATCHES Release )
    #message(" * Release Mode")
    add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo )
    #message(" * Release with Debug Info Mode")
    add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug )
    message(" * Debug Mode")
endif()

#############################################
# Dependencies

find_package(OpenCV REQUIRED)
find_package(ZED 3 REQUIRED)
find_package(CUDA REQUIRED)

set(DEPENDENCIES_COMP
    rclcpp
    rclcpp_components
    builtin_interfaces
    sensor_msgs
    image_transport
)

set(DEPENDENCIES_CONT
    rclcpp
    rclcpp_components
    zed_components
)

find_package(ament_cmake REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(zed_components)

###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

###############################################################################
# INCLUDES and LIBS
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}
)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

###############################################################################
# SOURCES

set(INC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/visibility_control.h
)

set(INC_COMP
    ${CMAKE_CURRENT_SOURCE_DIR}/src/component/include/zed_rgb_convert_component.hpp
)

set(SRC_COMP
    ${CMAKE_CURRENT_SOURCE_DIR}/src/component/src/zed_rgb_convert_component.cpp
)

set(SRC_CONT
    ${CMAKE_CURRENT_SOURCE_DIR}/src/container/zed_rgb_convert.cpp
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
target_link_libraries(${PROJECT_NAME}_component
    ${OpenCV_LIBS}
)
ament_target_dependencies(${PROJECT_NAME}_component
    ${DEPENDENCIES_COMP}
)

# Container
add_executable(${PROJECT_NAME}
    ${SRC_CONT}
)
target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_component
)
ament_target_dependencies(${PROJECT_NAME}
    ${DEPENDENCIES_CONT}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

rclcpp_components_register_nodes(${PROJECT_NAME}_component "stereolabs::ZedRgbCvtComponent")
set(node_plugins "${node_plugins}stereolabs::ZedRgbCvtComponent;$<TARGET_FILE:zed_rgb_convert_component>\n")

# Install components
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/include/
    DESTINATION include/${PROJECT_NAME}/
)

# Install LAUNCH files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_component)
ament_export_dependencies(${DEPENDENCIES_CONT} ${DEPENDENCIES_COMP})
ament_package()
