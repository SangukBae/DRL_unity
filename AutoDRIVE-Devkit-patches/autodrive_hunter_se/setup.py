from setuptools import setup
import os
from glob import glob

package_name = 'autodrive_hunter_se'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'gevent', 'gevent-websocket', 'python-socketio[client]'],
    zip_safe=True,
    maintainer='SangukBae',
    maintainer_email='sangukbae@example.com',
    description='AutoDRIVE Ecosystem ROS 2 Package for AgileX Hunter SE',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autodrive_bridge = autodrive_hunter_se.autodrive_bridge:main',
        ],
    },
)
