cmake_minimum_required(VERSION 3.8)
project(zed_isaac_ros_object_detection)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install LAUNCH, CONFIG and Foxglove layout files
install(DIRECTORY
  launch config foxglove
  DESTINATION share/${PROJECT_NAME}
)

# Install the Python node that rescales the detections to the ZED image
install(PROGRAMS
  scripts/zed_detection_rescaler.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
