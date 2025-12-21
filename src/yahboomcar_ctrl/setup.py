from setuptools import setup

package_name = 'yahboomcar_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='info@yahboom.com',
    description='Yahboom car control package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yahboom_joy_X3 = yahboomcar_ctrl.yahboom_joy_X3:main',
            'yahboom_keyboard = yahboomcar_ctrl.yahboom_keyboard:main',
        ],
    },
)
