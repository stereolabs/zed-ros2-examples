#!/bin/bash

################################ SOURCE ENVIRONMENT ################################################

#source ROS Humble
source /opt/ros/humble/setup.sh

#source workspace (assuming you're launching the terminal from the workspace folder)
source install/setup.sh

#####################################################################################################

############################################## PARAMETERS ###########################################

# Sleep time (in seconds) between tests
SLEEP_DURATION=10  

#Performance Test duration (in seconds)
TEST_DURATION=20  

# Package name and config filename
ROS_PACKAGE_NAME="zed_ros2_wrapper_qa"

# Get the package's share directory
PACKAGE_SHARE_DIR=$(ros2 pkg prefix --share $ROS_PACKAGE_NAME 2>/dev/null)

# Check if the package exists
if [ -z "$PACKAGE_SHARE_DIR" ]; then
    echo "Package '$ROS_PACKAGE_NAME' not found!"
    exit 1
fi

# Get the package's results directory in which to save results text files
RESULTS_DIR="$PACKAGE_SHARE_DIR/results/performance_test_results"

# Get the camera types (no space between types)
CAMERA_TYPES='[zedx]' #one camera (can be 'zedx', 'zedxm' for ZED X Mini, 'zed2i', 'zedm' for ZED Mini, 
CAMERA_TYPES_2='[zedx,zedx]' #two cameras
CAMERA_TYPES_4='[zedx,zedx,zedx,zedx]' #four cameras

# Get the camera names (no space between names)
CAMERA_NAMES='[zed_front]'
CAMERA_NAMES_2='[zed_front,zed_back]' #2 cameras - assuming front and back positions
CAMERA_NAMES_4='[zed_front,zed_back,zed_left,zed_right]' #4 cameras - assuming front, back, left and right positions

# Get the camera serial numbers (no space between serial numbers)
CAMERA_SN='[41758409]' #1 camera
CAMERA_SN_2='[41758409,49479313]' #2 cameras
CAMERA_SN_4='[41758409,49479313,40799433,50543054]' #4 cameras

# Use RVIZ 
USE_RVIZ='false'

# Use Image 
USE_IMAGE='true'

# Use Object Detection
USE_OBJ='true'

# Use Pointcloud 
USE_POINTCLOUD='true'

# URDF mode ('mono' for 1 cam config, 'duo' for 2 cam config, 'quad' for 4 cam config)
URDF_MODE='mono'
URDF_MODE_2='duo'
URDF_MODE_4='quad'

RVIZ_CONFIG_FILENAME_TEST="$PACKAGE_SHARE_DIR/rviz/ros_wrapper_performance_test_viewer.rviz"

#####################################################################################################

########################################### EXAMPLE TESTS ######################################


CONFIG_FILENAME="ros_wrapper_performance_test_config.yaml"  # Change this to your config file name, it can be changed to make different test types (with just image capture, ar additional depth, positional 
#tracking or object detection modules)

TEST_CONFIG_PATH="$PACKAGE_SHARE_DIR/config/$CONFIG_FILENAME"

# Check if the config file was found
if [ -n "$TEST_CONFIG_PATH" ]; then
    echo "Config file found: $TEST_CONFIG_PATH"
else
    echo "Config file '$CONFIG_FILENAME' not found in package '$PACKAGE_NAME'."
    exit 1
fi

echo "############################### TEST 1 : One Camera ##################################"

# Launch the node in the background
ros2 launch zed_ros2_wrapper_qa zed_wrapper_performance_test.launch.py \
    cam_names:=$CAMERA_NAMES cam_models:=$CAMERA_TYPES cam_serials:=$CAMERA_SN \
    shutdown_time:=$TEST_DURATION \
    results_file_path:=$RESULTS_DIR \
    test_name:='ONE_CAMERA' \
    ros_params_override_path:=$TEST_CONFIG_PATH\
    use_image:=$USE_IMAGE \
    use_pointcloud:=$USE_POINTCLOUD  \
    use_rviz:=$USE_RVIZ \
    rviz_config_file:=$RVIZ_CONFIG_FILENAME_TEST \
    urdf_mode:=$URDF_MODE &
   
# Get process ID of the last background command 
NODE_PID=$!  
echo "Node started with PID $NODE_PID"

# Wait for the node to finish before continuing
wait $NODE_PID
echo "Node with PID $NODE_PID has exited."
    
# Sleep before next iteration
echo "Sleeping for $SLEEP_DURATION seconds before next test..."
sleep $SLEEP_DURATION

echo "############################### TEST 2 : Two Cameras ##################################"

# Launch the node in the background
ros2 launch zed_ros2_wrapper_qa zed_wrapper_performance_test.launch.py \
    cam_names:=$CAMERA_NAMES_2 cam_models:=$CAMERA_TYPES_2 cam_serials:=$CAMERA_SN_2 \
    shutdown_time:=$TEST_DURATION \
    results_file_path:=$RESULTS_DIR \
    test_name:='TWO_CAMERAS' \
    ros_params_override_path:=$TEST_CONFIG_PATH \
    use_image:=$USE_IMAGE \
    use_pointcloud:=$USE_POINTCLOUD \
    use_rviz:=$USE_RVIZ \
    rviz_config_file:=$RVIZ_CONFIG_FILENAME_TEST \
    urdf_mode:=$URDF_MODE_2 &
   
# Get process ID of the last background command 
NODE_PID=$!  
echo "Node started with PID $NODE_PID"

# Wait for the node to finish before continuing
wait $NODE_PID
echo "Node with PID $NODE_PID has exited."
sleep $SLEEP_DURATION

echo "############################### TEST 3 : Four Cameras ##################################"

# Launch the node in the background
ros2 launch zed_ros2_wrapper_qa zed_wrapper_performance_test.launch.py \
    cam_names:=$CAMERA_NAMES_4 cam_models:=$CAMERA_TYPES_4 cam_serials:=$CAMERA_SN_4 \
    shutdown_time:=$TEST_DURATION \
    results_file_path:=$RESULTS_DIR \
    test_name:='FOUR_CAMERAS' \
    ros_params_override_path:=$TEST_CONFIG_PATH \
    use_image:=$USE_IMAGE \
    use_pointcloud:=$USE_POINTCLOUD \
    use_rviz:=$USE_RVIZ \
    rviz_config_file:=$RVIZ_CONFIG_FILENAME_TEST \
    urdf_mode:=$URDF_MODE_4 &
   
# Get process ID of the last background command 
NODE_PID=$!  
echo "Node started with PID $NODE_PID"

# Wait for the node to finish before continuing
wait $NODE_PID
echo "Node with PID $NODE_PID has exited."
sleep $SLEEP_DURATION


echo "All tests have been completed!"
