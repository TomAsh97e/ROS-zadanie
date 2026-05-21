from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'hydrolab_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*')),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mateusz',
    maintainer_email='matblaz12345@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pilot = hydrolab_mission.pilot_node:main',
            'behavior_tree = hydrolab_mission.mission_behavior_tree_node:main',
            'raycasted_approach_coordinator_node = '
            'hydrolab_mission.raycasted_approach_coordinator_node:main',
            'hover_controller = '
            'hydrolab_mission.hover_controller_node:main',
            'gimbal_controller = '
            'hydrolab_mission.gimbal_controller_node:main',
            'gimbal_tracker = '
            'hydrolab_mission.gimbal_tracker_node:main',
            'ball_detector = '
            'hydrolab_mission.ball_detector_node:main',
            'ball_grabber = '
            'hydrolab_mission.ball_grabber_node:main',
        ],
    },
)
