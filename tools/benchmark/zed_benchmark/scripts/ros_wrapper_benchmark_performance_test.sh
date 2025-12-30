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
TEST_DURATION=30  

# Package name and config filename
ROS_PACKAGE_NAME="zed_topic_benchmark"

# Get the package's share directory
PACKAGE_SHARE_DIR=$(ros2 pkg prefix --share $ROS_PACKAGE_NAME 2>/dev/null)

# Check if the package exists
if [ -z "$PACKAGE_SHARE_DIR" ]; then
    echo "Package '$ROS_PACKAGE_NAME' not found!"
    exit 1
fi

# Get the package's results directory in which to save results text files
RESULTS_FILE_PATH="results.txt"

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

#####################################################################################################

########################################### EXAMPLE TESTS ###########################################


CONFIG_FILENAME="ros_wrapper_performance_test_config.yaml"  # Change this to your config file name, it can be changed to make different test types (with just image capture, ar additional depth, positional 
# tracking or object detection modules)

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
ros2 launch zed_topic_benchmark zed_wrapper_benchmark_test.launch.py \
    cam_names:=$CAMERA_NAMES cam_models:=$CAMERA_TYPES cam_serials:=$CAMERA_SN \
    performance_test_duration:=$TEST_DURATION \
    results_file_path:="results_1_cam.txt" \
    ros_params_override_path:=$TEST_CONFIG_PATH\
   
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
ros2 launch zed_topic_benchmark zed_wrapper_benchmark_test.launch.py \
    cam_names:=$CAMERA_NAMES_2 cam_models:=$CAMERA_TYPES_2 cam_serials:=$CAMERA_SN_2 \
    performance_test_duration:=$TEST_DURATION \
    results_file_path:="results_2_cams.txt" \
    ros_params_override_path:=$TEST_CONFIG_PATH\
   
# Get process ID of the last background command 
NODE_PID=$!  
echo "Node started with PID $NODE_PID"

# Wait for the node to finish before continuing
wait $NODE_PID
echo "Node with PID $NODE_PID has exited."
sleep $SLEEP_DURATION

echo "############################### TEST 3 : Four Cameras ##################################"

# Launch the node in the background
ros2 launch zed_topic_benchmark zed_wrapper_benchmark_test.launch.py \
    cam_names:=$CAMERA_NAMES_4 cam_models:=$CAMERA_TYPES_4 cam_serials:=$CAMERA_SN_4 \
    performance_test_duration:=$TEST_DURATION \
    results_file_path:="results_4_cams.txt" \
    ros_params_override_path:=$TEST_CONFIG_PATH\
   
# Get process ID of the last background command 
NODE_PID=$!  
echo "Node started with PID $NODE_PID"

# Wait for the node to finish before continuing
wait $NODE_PID
echo "Node with PID $NODE_PID has exited."

echo "All tests have been completed!"
