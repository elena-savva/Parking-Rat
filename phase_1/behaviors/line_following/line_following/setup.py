import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'line_following'

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
    description='Line-following behavior: PID steering from camera line detection.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_tracker   = line_following.line_tracker_node:main',
            'angular_tracker = line_following.angular_tracker_node:main',
            'navigator = line_following.navigator_node:main',
        ],
    },
)
