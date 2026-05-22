import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _resolve_config_path(explicit_path, config_dir, file_name):
    if explicit_path:
        return explicit_path
    if config_dir:
        return os.path.join(config_dir, file_name)
    raise RuntimeError(
        f'Missing required config for {file_name}. '
        'Set config_dir (or HYDROLAB_CONFIG_DIR) or provide explicit launch argument.'
    )


def _require_file_exists(path):
    if not os.path.isfile(path):
        raise RuntimeError(f'Required config file not found: {path}')


def _launch_setup(context, *args, **kwargs):
    pkg_name = 'hydrolab_mission'
    config_dir = LaunchConfiguration('config_dir').perform(context).strip()
    mission_params_override = LaunchConfiguration('mission_params').perform(context).strip()
    camera_params_override = LaunchConfiguration('camera_config').perform(context).strip()

    mission_params_file = _resolve_config_path(
        mission_params_override,
        config_dir,
        'mission_params.yaml',
    )
    camera_params_file = _resolve_config_path(
        camera_params_override,
        config_dir,
        'camera_config.yaml',
    )
    _require_file_exists(mission_params_file)
    _require_file_exists(camera_params_file)

    pilot_node = Node(
        package=pkg_name,
        executable='pilot',
        name='pilot_node',
        output='screen',
        parameters=[mission_params_file],
    )

    raycasted_approach_coordinator_node = Node(
        package=pkg_name,
        executable='raycasted_approach_coordinator_node',
        name='raycasted_approach_coordinator_node',
        output='screen',
        parameters=[mission_params_file],
    )

    hover_controller_node = Node(
        package=pkg_name,
        executable='hover_controller',
        name='hover_controller',
        output='screen',
        parameters=[mission_params_file],
    )

    gimbal_controller_node = Node(
        package=pkg_name,
        executable='gimbal_controller',
        name='gimbal_controller',
        output='screen',
        parameters=[mission_params_file, camera_params_file],
    )

    gimbal_tracker_node = Node(
        package=pkg_name,
        executable='gimbal_tracker',
        name='gimbal_tracker',
        output='screen',
        parameters=[mission_params_file, camera_params_file],
    )

    ball_detector_node = Node(
        package=pkg_name,
        executable='ball_detector',
        name='ball_detector',
        output='screen',
        parameters=[mission_params_file],
    )

    ball_grabber_node = Node(
        package=pkg_name,
        executable='ball_grabber',
        name='ball_grabber',
        output='screen',
        parameters=[mission_params_file],
    )

    behavior_tree_node = Node(
        package=pkg_name,
        executable='behavior_tree',
        name='mission_behavior_tree',
        output='screen',
        parameters=[mission_params_file],
    )

    return [
        pilot_node,
        raycasted_approach_coordinator_node,
        hover_controller_node,
        gimbal_controller_node,
        gimbal_tracker_node,
        ball_detector_node,
        ball_grabber_node,
        behavior_tree_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_dir',
            default_value=EnvironmentVariable('HYDROLAB_CONFIG_DIR', default_value=''),
            description=(
                'Directory containing mission_params.yaml and camera_config.yaml. '
                'Required unless mission_params and camera_config are both set explicitly.'
            ),
        ),
        DeclareLaunchArgument(
            'mission_params',
            default_value='',
            description='Optional full path to mission_params.yaml',
        ),
        DeclareLaunchArgument(
            'camera_config',
            default_value='',
            description='Optional full path to camera_config.yaml',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
