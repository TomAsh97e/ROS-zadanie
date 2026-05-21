import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'test_mission'
    params_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'barrel_drop_params.yaml',
    )

    pilot_node = Node(
        package='last_mile_logistics_mission',
        executable='pilot',
        name='pilot_node',
        output='screen',
        parameters=[params_file],
    )

    mission_node = Node(
        package=pkg_name,
        executable='barrel_drop_mission',
        name='barrel_drop_mission',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        pilot_node,
        mission_node,
    ])
