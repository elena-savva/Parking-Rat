import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Motor controller with gain and trim calibration.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics_node = motor_control.kinematics_node:main',
            'odometry_node = motor_control.odometry_node:main',
            'pid_controller = motor_control.pid_controller_node:main', 
            'hand_follower = motor_control.hand_follower_node:main',
            'movement_node = motor_control.movement_node:main',
        ],
    },
)