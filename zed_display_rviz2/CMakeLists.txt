cmake_minimum_required(VERSION 3.5)

project(zed_display_rviz2)

find_package(ament_cmake REQUIRED)

###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

###############################################################################
# INSTALL

# Install LAUNCH files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  rviz2
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
