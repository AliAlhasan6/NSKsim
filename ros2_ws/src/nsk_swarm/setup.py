from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nsk_swarm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # World files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # RViz configs
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ali',
    maintainer_email='ali@example.com',
    description='NSK Swarm Robotics 2D Simulation package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_node         = nsk_swarm.robot_node:main',
            'convergence_monitor = nsk_swarm.convergence_monitor:main',
        ],
    },
)
