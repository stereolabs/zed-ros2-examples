<h1 align="center">
   <img src="./images/Picto+STEREOLABS_Black.jpg" alt="Stereolabs" title="Stereolabs" /><br \>
   ROS2 wrapper QA 
</h1>

<p align="center">
  ROS2 Wrapper QA package for testing the ZED ROS2 wrapper performances.<br>
  ROS2 Foxy Fitzroy (Ubuntu 20.04) - ROS2 Humble Hawksbill (Ubuntu 22.04)
</p>

<hr>

This package is intended for advanced users, QA engineers, and developers who want to evaluate the performance characteristics of the ZED ROS2 wrapper under controlled conditions.
It is designed to benchmark the performance of the *ZED ROS2 wrapper* in a controlled and repeatable manner, in use with single or multi ZED cameras setups. Its primary goal is to quantify the computational and runtime impact of the wrapper across a range of realistic operating conditions, providing clear insight into how different components of the ZED SDK pipeline behave on a given system. The package collects and compares key performance metrics such as CPU and GPU load, image and point cloud publishing frame rates. It also enables systematic comparisons between visualization-enabled and headless configurations (e.g., Rviz enabled vs. disabled), allowing users to isolate the overhead introduced by rendering and data transport. 

## Installation

### Prerequisites

- [Ubuntu 20.04 (Focal Fossa)](https://releases.ubuntu.com/focal/) or [Ubuntu 22.04 (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/)
- [ZED SDK](https://www.stereolabs.com/developers/release/latest/) v5.0 (for older versions support please check the [releases](https://github.com/stereolabs/zed-ros2-wrapper/releases))
- [CUDA](https://developer.nvidia.com/cuda-downloads) dependency
- ROS2 Foxy Fitzroy or ROS2 Humble Hawksbill: 
  - [Foxy on Ubuntu 20.04](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html) -> Close to EOL
  - [Humble on Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Linux-Install-Debians.html)
- [ZED ROS2 wrapper](https://github.com/stereolabs/zed-ros2-wrapper/) 

### Build the package

The **zed_ros2_wrapper_qa** is a [colcon](http://design.ros2.org/articles/build_tool.html) package. It requires that the **zed_ros2_wrapper** package is built in the ROS2 workspace. 

> :pushpin: **Note:** If you haven’t set up your colcon workspace yet, please follow this short [tutorial](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/).

To install the **zed_ros2_wrapper_qa**, copy the package in your ROS2 workspace, and run the following commands:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y # install dependencies
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc # automatically source the installation in every new bash (optional)
source ~/.bashrc
```

> :pushpin: **Note:** If `rosdep` is missing you can install it with:
>
>`sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstool python3-rosinstall build-essential`

> :pushpin: **Note:** When using the ZED ROS2 Wrapper on an NVIDIA Jetson with JP6, it is possible that you get the following error when building the package for the first time
>
> ```
> CMake Error at /usr/share/cmake-3.22/Modules/FindCUDA.cmake:859 (message):
>   Specify CUDA_TOOLKIT_ROOT_DIR
> Call Stack (most recent call first):
>  /usr/local/zed/zed-config.cmake:72 (find_package)
>  CMakeLists.txt:81 (find_package)
> ```
>
> You can fix the problem by installing the missing `nvidia-jetpack` packages:
>
> `sudo apt install nvidia-jetpack nvidia-jetpack-dev`
>
> :pushpin: **Note:** The option `--symlink-install` is very important, it allows to use symlinks instead of copying files to the ROS2 folders during the installation, where possible. Each package in ROS2 must be installed and all the files used by the nodes must be copied into the installation folders. Using symlinks allows you to modify them in your workspace, reflecting the modification during the next executions without needing to issue a new `colcon build` command. This is true only for all the files that don't need to be compiled (Python scripts, configurations, etc.).

> :pushpin: **Note:** To guarantee optimal results with the ROS2 wrapper, it is recommended to follow this [tutorial](https://www.stereolabs.com/docs/ros2/dds_and_network_tuning).


## Package Architecture

### Available Launch Files

Two launch files are provided to run performance evaluations of the ZED ROS2 wrapper, depending on the desired test configuration.

#### zed_wrapper_performance_test.launch.py

This is the recommended launcher for IPC-based multi-camera performance testing. It starts a multi-camera ZED ROS2 wrapper configuration together with the performance evaluation node, all within the same container.

Camera parameters are defined in the same way as in the official [multi-camera tutorial](https://www.stereolabs.com/docs/ros2/125_multi_camera).

Depending on the number of cameras used, the urdf_mode parameter must be set appropriately:
- "mono" for single camera setups
- "duo" for two-camera setups
- "quad" for four-camera setups

This ensures that the system loads the correct static TF configuration.

Wrapper parameters can be customized using the **ros_params_override_path** argument, allowing users to run performance tests with any desired ZED ROS2 wrapper configuration. The duration of the test is controlled by the **Performance Test Duration Node**, which is launched as a required node. Once the specified test duration elapses, this node is terminated, automatically shutting down all other nodes and components started by the launcher (i.e., the performance node and the ZED ROS2 wrapper instances).

#### zed_multi_camera.launch.py

This launcher is used to start the ZED ROS2 wrapper and the performance node within the same container, and is compatible with both single-camera and multi-camera setups. 

### Available Nodes

#### Performance Node 

The core of the package is a composable node class named PerformanceTest. This node subscribes to the relevant ZED ROS2 wrapper topics (image and/or point cloud streams) in either single-camera or multi-camera configurations. As data is published by the cameras, the node continuously gathers performance metrics such as frame rates and system load. When the test ends, all collected metrics are aggregated and written to a JSON file, providing a structured and reproducible summary of the test results.

The JSON file contains:
- Average and peak CPU usage
- Average and peak GPU usage
- Image FPS per camera
- Point cloud FPS per camera
- Test metadata (test name, duration, camera count, RViz usage)

List of parameters : 
- __test_name__ (string): name of the current test, can be used to give more information and context about the performance test setup (ie: "PCL ONLY - 15 FPS - NO Rviz")
- __cam_count__ (integer): number of cameras used in the multicam configuration setup
- __use_image__ (bool): whether to subscribe to image topics for the test
- __use_pointcloud__ (bool): whether to subscribe to pointcloud topics for the test
- __use_rviz__ (bool): whether to use Rviz for the test (displaying data has an impact on performances)

#### Performance test duration node

The Performance Test Duration Node controls the overall duration of a performance run. It takes a time parameter specifying how long the test should execute.This node is marked as required in the launch file. When the specified duration is reached, the node is terminated, which in turn triggers the shutdown of all other nodes and components launched as part of the performance test. This mechanism ensures clean and repeatable test execution without manual intervention.

### Available URDFs files

URDFs files can be used to set up the static transforms between cameras - if needed (extrinsic calibration). They do not have an influence over performances but a proper TF setup ensure a correct visualization of the pointclouds in Rviz. 

Three different example files templates available : 

- "mono" for single camera setups
- "duo" for two-camera setups
- "quad" for four-camera setups


### Available Scripts

Bash scripts can be used to set up automated successive performance tests. A commented example is available in the `script` folder, ros_wrapper_performance_test.sh. It provides a template for single camera and multi camera setup performance tests, performed successively.

## Tutorial: create a performance test script 

### Step 1 : Configure your camera setup 

Connect your ZED cameras to your device. If you need to set up the static TF transforms, create or modify the URDF file present in the `urdf` folder of the package and add the camera extrinsic calibration. 

Example of multi camera setup (dual camera setup, one camera at the front and one at the rear - yaw rotation of Pi radian): 

```xml
<robot name="zed_multi_camera" xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:property name="M_PI"     value="3.1415926535897931" />
  
  <!-- Define the required parameters -->
  <xacro:arg name="multi_link" default="zed_multi_link" />
  <xacro:arg name="camera_name_0" default="zed_front" />
  <xacro:arg name="camera_name_1" default="zed_rear" />

  <!-- Define the reference links -->
  <link name="$(arg multi_link)" />
  <link name="$(arg camera_name_0)_camera_link" />
  <link name="$(arg camera_name_1)_camera_link" />

  <!-- Note the URDF of the cameras are loaded by the respective launch files.
       We must only provide the position of each of them with respect to the reference link -->

  <!-- Define the position of the front camera with respect to the reference link -->
  <joint name="$(arg camera_name_0)_camera_joint" type="fixed">
      <parent link="$(arg camera_name_0)_camera_link"/>
      <child link="$(arg multi_link)"/>
      <origin xyz="-0.06 0.0 0.0" rpy="0 0 0" />
  </joint>

  <!-- Define the position of the rear camera with respect to the reference link -->
  <joint name="$(arg camera_name_1)_camera_joint" type="fixed">
      <parent link="$(arg multi_link)"/>
      <child link="$(arg camera_name_1)_camera_link"/>
      <origin xyz="-0.06 0.0 0.0" rpy="0 0 ${M_PI}" />
  </joint>

  <!-- NOTE: Replicate the same XML code above for all the cameras of the multi-camera system if more cameras are used -->
  
</robot>
```

Translation and rotation arguments can be modified to fit your setup. Additional cameras can also be added. 

> :pushpin: **Optional:** The Rviz file can also be created in the `rviz` folder. The panel can be tuned once and saved when the first test is performed so you can add the topics you wish to visualize (pointclouds, images...).

### Step 2: Configure your ROS2 Wrapper parameters

Modify or create a new yaml config file present in the `config` folder of the package. Help yourself with the [Stereolabs documentation](https://www.stereolabs.com/docs) to adjust parameters of the various SDK module to suit your needs. 

Example of configuration (ZED X, HD1200, 30 FPS with depth using NEURAL with GEN3 positional Tracking and no object detection):

```yaml
# config/common_stereo.yaml
# Common parameters to Stereolabs ZED Stereo cameras

---
/**:
    ros__parameters:

        general:
            pub_resolution: 'CUSTOM' # The resolution used for image and depth map publishing. 'NATIVE' to use the same `general.grab_resolution` - `CUSTOM` to apply the `general.pub_downscale_factor` downscale factory to reduce bandwidth in transmission
            pub_downscale_factor: 2.0 # rescale factor used to rescale image before publishing when 'pub_resolution' is 'CUSTOM'
            pub_frame_rate: 30.0 # frequency of publishing of visual images and depth images
            camera_model: 'zedx'
            grab_resolution: 'HD1200' # The native camera grab resolution. 'HD1200', 'HD1080', 'SVGA', 'AUTO'
            grab_frame_rate: 30 # ZED SDK internal grabbing rate (HD1200/HD1080: 60, 30, 15 - SVGA: 120, 60, 30, 15)

        depth: # Only stereo cameras
            depth_mode: 'NEURAL' # Matches the ZED SDK setting: 'NONE', 'PERFORMANCE', 'QUALITY', 'ULTRA', 'NEURAL', 'NEURAL_PLUS' - Note: if 'NONE' all the modules that requires depth extraction are disabled by default (Pos. Tracking, Obj. Detection, Mapping, ...)
            point_cloud_freq: 30.0 # [DYNAMIC] - frequency of the pointcloud publishing (equal or less to `pub_frame_rate` value)
            point_cloud_res: 'COMPACT' # The resolution used for point cloud publishing - 'COMPACT'-Standard resolution. Optimizes processing and bandwidth, 'REDUCED'-Half resolution. Low processing and bandwidth requirements
            depth_stabilization: 30 # [DYNAMIC] - enable/disable depth stabilization. 0: disabled, 1: enabled. Note: this parameter is not available for the ZED camera

        pos_tracking:
            pos_tracking_enabled: true # True to enable positional tracking from start
            pos_tracking_mode: 'GEN_3' # Matches the ZED SDK setting: 'GEN_1', 'GEN_2'
            imu_fusion: true # enable/disable IMU fusion. When set to false, only the optical odometry will be used.
            publish_tf: true # [usually overwritten by launch file] publish `odom -> camera_link` TF
            publish_map_tf: true # [usually overwritten by launch file] publish `map -> odom` TF

        object_detection:
            od_enabled: false # True to enable Object Detection
            enable_tracking: true # Whether the object detection system includes object tracking capabilities across a sequence of images.
            detection_model: 'MULTI_CLASS_BOX_FAST' # 'MULTI_CLASS_BOX_FAST', 'MULTI_CLASS_BOX_MEDIUM', 'MULTI_CLASS_BOX_ACCURATE', 'PERSON_HEAD_BOX_FAST', 'PERSON_HEAD_BOX_ACCURATE', 'CUSTOM_YOLOLIKE_BOX_OBJECTS'
            max_range: 20.0 # [m] Upper depth range for detections.The value cannot be greater than 'depth.max_depth'
            filtering_mode: 'NMS3D' # Filtering mode that should be applied to raw detections: 'NONE', 'NMS3D', 'NMS3D_PER_CLASS'
            prediction_timeout: 2.0 # During this time [sec], the object will have OK state even if it is not detected. Set this parameter to 0 to disable SDK predictions
            allow_reduced_precision_inference: false # Allow inference to run at a lower precision to improve runtime and memory usage
            # Other parameters are defined in the 'object_detection.yaml' and 'custom_object_detection.yaml' files
```

### Step 3: Configure your Performance Test script

Modify or create a new yaml bash script file present in the `scripts` folder of the package and tune the parameters to suit your needs. 

Example of performance test script configuration (2 performance tests with different ROS2 Wrapper configurations) :

```bash
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

# Perfromance Test duration (in seconds) 
TEST_DURATION=30   

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

# Get the camera names (no space between names)
CAMERA_NAMES='[zed_front]'
CAMERA_NAMES_2='[zed_front,zed_back]' #2 cameras - assuming front and back positions

# Get the camera serial numbers (no space between serial numbers)
CAMERA_SN='[41758409]' #1 camera
CAMERA_SN_2='[41758409,49479313]' #2 cameras

# Use RVIZ 
USE_RVIZ='true'

# Use Image 
USE_IMAGE='true'

# Use Object Detection
USE_OBJ='false'

# Use Pointcloud 
USE_POINTCLOUD='true'

# URDF mode ('mono' for 1 cam config, 'duo' for 2 cam config, 'quad' for 4 cam config)
URDF_MODE='mono'
URDF_MODE_2='duo'
URDF_MODE_4='quad'

RVIZ_CONFIG_FILENAME_TEST="$PACKAGE_SHARE_DIR/rviz/ros_wrapper_performance_test_viewer.rviz"

#####################################################################################################

########################################### EXAMPLE TESTS ######################################

echo "############################### TEST 1 : One Camera ##################################"

CONFIG_FILENAME="ros_wrapper_performance_test_config_test1.yaml"  # Change this to your config file name, it can be changed to make different test types (with just image capture, ar additional depth, positional 
#tracking or object detection modules)

TEST_CONFIG_PATH="$PACKAGE_SHARE_DIR/config/$CONFIG_FILENAME"

# Check if the config file was found
if [ -n "$TEST_CONFIG_PATH" ]; then
    echo "Config file found: $TEST_CONFIG_PATH"
else
    echo "Config file '$CONFIG_FILENAME' not found in package '$PACKAGE_NAME'."
    exit 1
fi

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

echo "############################### TEST 2 : Two Cameras and different ROS2 wrapper config ##################################"

CONFIG_FILENAME="ros_wrapper_performance_test_config_test2.yaml"  # Change this to your config file name, it can be changed to make different test types (with just image capture, ar additional depth, positional 
#tracking or object detection modules)

TEST_CONFIG_PATH="$PACKAGE_SHARE_DIR/config/$CONFIG_FILENAME"

# Check if the config file was found
if [ -n "$TEST_CONFIG_PATH" ]; then
    echo "Config file found: $TEST_CONFIG_PATH"
else
    echo "Config file '$CONFIG_FILENAME' not found in package '$PACKAGE_NAME'."
    exit 1
fi

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

echo "All tests have been completed!"
```

#### Step 4: Launch the Performance test

Recompile your ROS2 workspace and launch the bash script

```
cd ros_ws
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
source install/setup.bash
bash src/zed-ros2-wrapper-qa/scripts/performance_test_script.sh
```

At the end of the tests, retrieve the results in the directory mentioned as input parameter of the Performance test. 
