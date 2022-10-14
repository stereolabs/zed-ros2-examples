#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Start the Nav2 nodes when the ZED node is ready
    nav2_node_delay = 10.0

    # Start Rviz when the ZED Node is ready
    rviz_node_delay = nav2_node_delay + 5.0

    # Launch configuration variables (can be changed by CLI command)

    #############################################################################
    # ZED variables
    # Input SVO path
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
    #############################################################################

    #############################################################################
    # Nav2 variables
    nav2_params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    #############################################################################

    #############################################################################
    # Rviz2 variables
    #launch_rviz2 = LaunchConfiguration('rviz2')
    #############################################################################

    #############################################################################
    # ZED Launch arguments
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
    #############################################################################

    #############################################################################
    # Nav2 Launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory(
            'zed_costmap2d_example'), 'params', 'nav2_custom_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_distance.xml'),
        description='Full path to the behavior tree xml file to use')
    #############################################################################

    #############################################################################
    # Rviz2 Launch arguments
    #declare_rviz2_cmd = DeclareLaunchArgument(
    #    'rviz2',
    #    default_value='True',
    #    description='Whether run Rviz2 for data visualization')
    #############################################################################

    #############################################################################
    # ZED Wrapper node
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_costmap2d_example'),
            '/launch/include/zed2i_bringup.launch.py'
        ]),
        launch_arguments={
            'svo_path': svo_path,
            'cam_pos_x': cam_pos_x,
            'cam_pos_y': cam_pos_y,
            'cam_pos_z': cam_pos_z,
            'cam_roll': cam_roll,
            'cam_pitch': cam_pitch,
            'cam_yaw': cam_yaw
        }.items()
    )
    #############################################################################

    #############################################################################
    # Nav2 bringup

    map_file = os.path.join(get_package_share_directory(
        'zed_costmap2d_example'), 'map', 'empty.yaml')

    nav2_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'params_file': nav2_params_file,
            'declare_bt_xml_cmd': default_bt_xml_filename,
            'slam': 'False',
            'map': map_file
        }.items()
    )
    #############################################################################

    #############################################################################
    # Rviz2 node

    # Ready Rviz2 Configurations to be loaded
    config_rviz2 = os.path.join(
        get_package_share_directory('zed_costmap2d_example'),
        'config',
        'example_config.rviz'
    )

    rviz2_node = Node(
        package='rviz2',
        #namespace='zed2i',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[["-d"], [config_rviz2]],
        #condition=IfCondition(launch_rviz2)
    )
    #############################################################################

    #############################################################################
    # Delay Action for ZED Node and Rviz2
    delayed_nav2 = TimerAction(
        period=nav2_node_delay,
        actions=[nav2_launch]
    )

    delayed_rviz_node = TimerAction(
        period=rviz_node_delay,
        actions=[rviz2_node],
        #condition=IfCondition(launch_rviz2)
    )
    #############################################################################

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
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)

    # Add nodes to LaunchDescription
    ld.add_action(zed_wrapper_launch)
    ld.add_action(delayed_nav2)
    ld.add_action(delayed_rviz_node)

    return ld
