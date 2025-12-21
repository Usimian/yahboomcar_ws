from setuptools import setup
import os
from glob import glob

package_name = 'yahboomcar_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name] if package_name in ['yahboomcar_description'] else [],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.STL'))),
        (os.path.join('share', package_name, 'meshes', 'sensor'), glob(os.path.join('meshes', 'sensor', '*.STL'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='info@yahboom.com',
    description='Yahboom car description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
