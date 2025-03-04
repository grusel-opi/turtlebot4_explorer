import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
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
    get_package_share_directory('turtlebot4_explorer'),
    'config',
    'explorer.yaml')

    explorer = Node(
        package="turtlebot4_explorer",
        executable="explorer",
        name="turtlebot4_explorer",
        parameters=[config]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(explorer)

    return ld

