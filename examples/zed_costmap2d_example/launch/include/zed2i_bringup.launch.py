#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Camera model
    # use:
    #  - 'zed' for "ZED" camera
    #  - 'zedm' for "ZED mini" camera
    #  - 'zed2' for "ZED2" camera
    #  - 'zed2i' for "ZED2i" camera
    camera_model = 'zed2i'

    # Launch configuration variables (can be changed by CLI command)
    svo_path = LaunchConfiguration('svo_path')

    # Position X of the camera with respect to the base frame [m].
    cam_pos_x = LaunchConfiguration('cam_pos_x')
    # Position Y of the camera with respect to the base frame [m].
    cam_pos_y = LaunchConfiguration('cam_pos_y')
    # Position Z of the camera with respect to the base frame [m].
    cam_pos_z = LaunchConfiguration('cam_pos_z')
    # Roll orientation of the camera with respect to the base frame [rad].
    cam_roll = LaunchConfiguration('cam_roll')
    # Pitch orientation of the camera with respect to the base frame [rad].
    cam_pitch = LaunchConfiguration('cam_pitch')
    # Yaw orientation of the camera with respect to the base frame [rad].
    cam_yaw = LaunchConfiguration('cam_yaw')

    # Configuration variables
    # Camera name. Can be different from camera model, used to distinguish camera in multi-camera systems
    camera_name = 'zed2i'
    node_name = 'zed_node'  # Zed Node name
    publish_urdf = 'true'  # Publish static frames from camera URDF
    # Robot base frame. Note: overrides the parameter `pos_tracking.base_frame` in `common.yaml`.
    base_frame = 'base_link'

    # ZED Configurations to be loaded by ZED Node
    config_common_path = os.path.join(
        get_package_share_directory('zed_costmap2d_example'),
        'params',
        'custom_common.yaml'
    )

    config_camera_path = os.path.join(
        get_package_share_directory('zed_costmap2d_example'),
        'params',
        'custom_' + camera_model + '.yaml'
    )

    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf', 'zed_descr.urdf.xacro'
    )

    # ZED Wrapper node
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/include/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_model': camera_model,
            'camera_name': camera_name,
            'node_name': node_name,
            'config_common_path': config_common_path,
            'config_camera_path': config_camera_path,
            'publish_urdf': publish_urdf,
            'xacro_path': xacro_path,
            'svo_path': svo_path,
            'base_frame': base_frame,
            'cam_pos_x': cam_pos_x,
            'cam_pos_y': cam_pos_y,
            'cam_pos_z': cam_pos_z,
            'cam_roll': cam_roll,
            'cam_pitch': cam_pitch,
            'cam_yaw': cam_yaw
        }.items()
    )

    declare_svo_path_cmd = DeclareLaunchArgument(
        'svo_path',
        # 'live' used as patch for launch files not allowing empty strings as default parameters
        default_value='live',
        description='Path to an input SVO file. Note: overrides the parameter `general.svo_file` in `common.yaml`.')

    declare_cam_pos_x = DeclareLaunchArgument(
        'cam_pos_x',
        default_value='0.0',
        description='X position of the camera with respect to `base_link`.')

    declare_cam_pos_y = DeclareLaunchArgument(
        'cam_pos_y',
        default_value='0.0',
        description='Y position of the camera with respect to `base_link`.')

    declare_cam_pos_z = DeclareLaunchArgument(
        'cam_pos_z',
        default_value='0.0',
        description='Z position of the camera with respect to `base_link`.')

    declare_cam_roll = DeclareLaunchArgument(
        'cam_roll',
        default_value='0.0',
        description='Roll rotation of the camera with respect to `base_link`.')

    declare_cam_pitch = DeclareLaunchArgument(
        'cam_pitch',
        default_value='0.0',
        description='Pitch rotation of the camera with respect to `base_link`.')

    declare_cam_yaw = DeclareLaunchArgument(
        'cam_yaw',
        default_value='0.0',
        description='Yaw rotation of the camera with respect to `base_link`.')
    
    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Launch parameters
    ld.add_action(declare_svo_path_cmd)
    ld.add_action(declare_cam_pos_x)
    ld.add_action(declare_cam_pos_y)
    ld.add_action(declare_cam_pos_z)
    ld.add_action(declare_cam_roll)
    ld.add_action(declare_cam_pitch)
    ld.add_action(declare_cam_yaw)

    # Add nodes to LaunchDescription
    ld.add_action(zed_wrapper_launch)

    return ld
