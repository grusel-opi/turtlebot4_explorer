import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


# yes, this is ugly but necessary..
def load_fukn_yaml(pkg_name, config_file_name, config_dir="config"):
    configFilepath = os.path.join(get_package_share_directory(pkg_name),
				  config_dir,
        	                  config_file_name)
    file = open(configFilepath, 'r')
    params = yaml.safe_load(file)['/oakd']['ros__parameters']
    return params


def launch_setup(context, *args, **kwargs):
    log_level = 'info'
    if(context.environment.get('DEPTHAI_DEBUG')=='1'):
        log_level='debug'

    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_descriptions'), 'launch')

    params_file_C = LaunchConfiguration("params_file_C")

    name_C = LaunchConfiguration('name_C').perform(context)

    parent_frame_C = LaunchConfiguration('parent_frame_C',  default = 'oak-d-C-base-frame')

    use_composition = LaunchConfiguration('rsp_use_composition', default='true')

    use_gdb      = LaunchConfiguration('use_gdb',       default = 'false')
    use_valgrind = LaunchConfiguration('use_valgrind',  default = 'false')
    use_perf     = LaunchConfiguration('use_perf',      default = 'false')

    launch_prefix = ''

    if (use_gdb.perform(context) == 'true'):
        launch_prefix += "gdb -ex run --args "
    if (use_valgrind.perform(context) == 'true'):
        launch_prefix += "valgrind --tool=callgrind"
    if (use_perf.perform(context) == 'true'):
        launch_prefix += "perf record -g --call-graph dwarf --output=perf.out.node_name.data --"
    return [
        ComposableNodeContainer(
            name=name_C+"_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::Camera",
                        name=name_C,
                        parameters=[load_fukn_yaml("oakd", "oakd.yaml")],
                    )
            ],
            arguments=['--ros-args', '--log-level', log_level],
            prefix=[launch_prefix],
            output="both",
        )
    ]


def generate_launch_description():
    oakd_pkg_prefix = get_package_share_directory("oakd")

    declared_arguments = [
        DeclareLaunchArgument("name_C", default_value="oakd_cam_C"),
        DeclareLaunchArgument("parent_frame_C", default_value="oak-d-C-base-frame"),
        DeclareLaunchArgument("params_file_C", default_value=os.path.join(oakd_pkg_prefix, 'config', 'cam_C.yaml')),
        DeclareLaunchArgument("rsp_use_composition", default_value='true'),
        DeclareLaunchArgument("use_gdb", default_value='false'),
        DeclareLaunchArgument("use_valgrind", default_value='false'),
        DeclareLaunchArgument("use_perf", default_value='false')
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )



