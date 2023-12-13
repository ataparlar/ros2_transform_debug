import os

from ament_index_python import get_package_share_directory

import launch

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    path_package = get_package_share_directory('leo_mapping_tools')

    point_cloud_origin_changer_params = os.path.join(path_package, 'config/point_cloud_origin_changer_params.yaml')
    point_cloud_origin_changer_node = Node(
        package='leo_mapping_tools',
        executable='point_cloud_origin_changer_exe',
        parameters=[point_cloud_origin_changer_params]
    )

    return launch.LaunchDescription(
        [point_cloud_origin_changer_node])
