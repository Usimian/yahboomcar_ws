from setuptools import setup
import os
from glob import glob

package_name = 'auto_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mw',
    maintainer_email='marc.wester@gmail.com',
    description='Autonomous driving package for yahboomcar with advanced navigation and decision-making capabilities',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_navigator = auto_drive.autonomous_navigator:main',
            'autonomous_control = auto_drive.autonomous_control:main',
            'debug_monitor = auto_drive.debug_monitor:main',
        ],
    },
)
