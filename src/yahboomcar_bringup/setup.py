from setuptools import setup
import os
from glob import glob

package_name = 'yahboomcar_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz*'))),
        (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nx-ros2',
    maintainer_email='nx-ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'Mcnamu_driver_X3 = yahboomcar_bringup.Mcnamu_driver_X3:main',
            'vel_raw_diagnostic = yahboomcar_bringup.vel_raw_diagnostic:main',
            'calibrate_linear_X3 = yahboomcar_bringup.calibrate_linear_X3:main', 
            'calibrate_angular_X3 = yahboomcar_bringup.calibrate_angular_X3:main',
            'test_calibration = yahboomcar_bringup.test_calibration:main',
            'robot_exercise = yahboomcar_bringup.robot_exercise:main',
        ],
    },
)
