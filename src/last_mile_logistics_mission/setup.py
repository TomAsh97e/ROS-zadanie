from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'last_mile_logistics_mission'

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
            'pilot = last_mile_logistics_mission.pilot_node:main',
            'behavior_tree = last_mile_logistics_mission.mission_behavior_tree_node:main',
            'gimbal_controller = '
            'last_mile_logistics_mission.gimbal_controller_node:main',
            'ball_detector = '
            'last_mile_logistics_mission.ball_detector_node:main',
            'ball_grabber = '
            'last_mile_logistics_mission.ball_grabber_node:main',
        ],
    },
)
