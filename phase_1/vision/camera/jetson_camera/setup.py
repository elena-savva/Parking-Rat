from setuptools import setup
from glob import glob
import os

package_name = 'jetson_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = jetson_camera.publisher_node:main',
            'subscriber_node = jetson_camera.subscriber_node:main',
            'processing_node = jetson_camera.processing_node:main',
            'pair_subscriber_node = jetson_camera.pair_subscriber_node:main',
            'calibrate_camera = jetson_camera.calibrate_camera:main',
        ],
    },
)
