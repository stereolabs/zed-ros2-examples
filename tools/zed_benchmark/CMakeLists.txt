cmake_minimum_required(VERSION 3.10)

project(topic_benchmark)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

###############################################################################
# Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(${PROJECT_NAME}_files SOURCES ${extra_files})
###############################################################################

## Sources
set(SRC_CONT src/topic_benchmark_container.cpp)
set(SRC_COMP src/component/src/topic_benchmark.cpp)

## Build
add_executable(${PROJECT_NAME} ${SRC_CONT} ${SRC_COMP})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

## Includes
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  src/component/include
)

# Install executable
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
