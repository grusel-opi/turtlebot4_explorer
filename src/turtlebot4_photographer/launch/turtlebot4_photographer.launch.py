import os
import yaml

import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


# yes, this is ugly but necessary..
def load_fukn_yaml(pkg_name, config_file_name, config_dir="config"):
    configFilepath = os.path.join(get_package_share_directory(pkg_name),
				  config_dir,
        	                  config_file_name)
    file = open(configFilepath, 'r')
    params = yaml.safe_load(file)['/'+pkg_name]['ros__parameters']
    return params


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
        name='turtlebot4_photographer_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='turtlebot4_photographer',
                plugin='turtlebot4_photographer::Photographer',
                name='turtlebot4_photographer',
                parameters=[load_fukn_yaml("turtlebot4_photographer", "turtlebot4_photographer.yaml")],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(photographer)

    return ld

