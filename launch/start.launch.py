"""Launch the rtk_base node.

Arguments: namespace (default 'base'), node_name (default 'rtk_base'),
params_file (default config/base_um982.yaml).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('rtk_base'),
                                  'config', 'base_um982.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='base'),
        DeclareLaunchArgument('node_name', default_value='rtk_base'),
        DeclareLaunchArgument('params_file', default_value=default_params),

        Node(
            package='rtk_base',
            executable='rtk_base_node',
            name=LaunchConfiguration('node_name'),
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
