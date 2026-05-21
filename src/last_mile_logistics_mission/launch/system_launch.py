import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'last_mile_logistics_mission'
    params_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'mission_params.yaml',
    )

    pilot_node = Node(
        package=pkg_name,
        executable='pilot',
        name='pilot_node',
        output='screen',
        parameters=[params_file],
    )

    gimbal_controller_node = Node(
        package=pkg_name,
        executable='gimbal_controller',
        name='gimbal_controller',
        output='screen',
        parameters=[params_file],
    )

    ball_detector_node = Node(
        package=pkg_name,
        executable='ball_detector',
        name='ball_detector',
        output='screen',
        parameters=[params_file],
    )

    ball_grabber_node = Node(
        package=pkg_name,
        executable='ball_grabber',
        name='ball_grabber',
        output='screen',
        parameters=[params_file],
    )

    behavior_tree_node = Node(
        package=pkg_name,
        executable='behavior_tree',
        name='mission_behavior_tree',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        pilot_node,
        gimbal_controller_node,
        ball_detector_node,
        ball_grabber_node,
        behavior_tree_node,
    ])
