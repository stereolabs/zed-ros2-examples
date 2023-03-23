from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default = False)

    zed_gnss_tutorial_config_path = LaunchConfiguration(
        'zed_gnss_tutorial_config_path',
        default = FindPackageShare('zed_gnss_tutorial').find('zed_gnss_tutorial') + '/config/zed_gnss_tutorial.yaml')

    zed_gnss_tutorial_node_container = ComposableNodeContainer(
            name = 'zed_gnss_tutorial_node_container',
            namespace = '',
            package = 'rclcpp_components',
            executable = 'component_container',
            composable_node_descriptions=[
                  ComposableNode(
                      package = 'zed_gnss_tutorial',
                      plugin = 'zed_gnss_tutorial::ZedGnssTutorialNode',
                      name = 'zed_gnss_tutorial_node',
                      parameters = [
                        #zed_gnss_tutorial_config_path,
                        {'use_sim_time': use_sim_time}
                        ],
                      extra_arguments = [{'use_intra_process_comms': True}]
                  ),
            ],
            output = 'both',
    )

    zed_gnss_tutorial_node_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='zed_gnss_tutorial_node_manager',
            output='both',
            parameters=[{'autostart': False},
                        {'node_names': ['zed_gnss_tutorial_node']},
                        {'bond_timeout': 4.0}],
            )

    return LaunchDescription([
        zed_gnss_tutorial_node_container,
        zed_gnss_tutorial_node_manager
    ])