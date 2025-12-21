from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slam_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mw',
    maintainer_email='marc.wester@gmail.com',
    description='SLAM and Navigation package for yahboomcar X3 with slam_toolbox and Nav2',
    license='MIT',
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        'console_scripts': [
            'initial_pose_publisher = slam_nav.initial_pose_publisher:main',
            'rgb_to_occupancy = slam_nav.rgb_to_occupancy:main',
            'camera_nav_monitor = slam_nav.camera_nav_monitor:main',
            'robot_interface_node = slam_nav.robot_interface_node:main',
            'calibration_test = slam_nav.calibration_test:main',
            'pointcloud_height_filter = slam_nav.pointcloud_height_filter:main',
        ],
    },
)
