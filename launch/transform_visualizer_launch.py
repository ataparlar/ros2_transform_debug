import os

from ament_index_python import get_package_share_directory

import launch

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    path_package = get_package_share_directory('leo_mapping_tools')

    transform_visualizer_param = os.path.join(path_package, 'config/transform_visualizer_params.yaml')
    transform_visualizer_node = Node(
        package='leo_mapping_tools',
        executable='transform_visualizer_exe',
        parameters=[transform_visualizer_param]
    )

    return launch.LaunchDescription(
        [transform_visualizer_node])
