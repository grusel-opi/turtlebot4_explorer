import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace')
]


def generate_launch_description():

    config = os.path.join(
    get_package_share_directory('turtlebot4_photographer'),
    'config',
    'turtlebot4_photographer.yaml')

    photographer = ComposableNodeContainer(
        name='turtlebot4_photographer',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='turtlebot4_photographer',
                plugin='turtlebot4_photographer::Photographer',
                name='turtlebot4_photographer',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(photographer)

    return ld

