from setuptools import setup
from glob import glob
import os

package_name = 'robotic_arm'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'),   glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='3-DOF robotic arm with obstacle avoidance and RViz2 visualization',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gui_node         = robotic_arm.gui_node:main',
            'ultrasonic_node  = robotic_arm.ultrasonic_node:main',
            'obstacle_node    = robotic_arm.obstacle_node:main',
            'planner_node     = robotic_arm.planner_node:main',
            'waypoint_node    = robotic_arm.waypoint_node:main',
            'ik_node          = robotic_arm.ik_node:main',
            'controller_node  = robotic_arm.controller_node:main',
            'ee_marker_node   = robotic_arm.ee_marker_node:main',
        ],
    },
)