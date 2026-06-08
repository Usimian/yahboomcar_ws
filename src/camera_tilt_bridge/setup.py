from setuptools import setup

package_name = 'camera_tilt_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mw',
    maintainer_email='marc.wester@gmail.com',
    description='Bridge /camera_tilt -> /bus_servo for the STS3215 tilt servo on the Yahboom board bus port.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_tilt_bridge = camera_tilt_bridge.camera_tilt_bridge:main',
        ],
    },
)
