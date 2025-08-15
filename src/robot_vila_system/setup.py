from setuptools import find_packages, setup

package_name = 'robot_vila_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_gateway_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mw',
    maintainer_email='marc.wester@gmail.com',
    description='ROS2 Robot VILA System - Single Gateway Architecture',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_client_node = robot_vila_system.robot_client_node:main',
            'gateway_validator_node = robot_vila_system.gateway_validator_node:main',
        ],
    },
)
