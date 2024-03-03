import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rktl_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('lib', package_name), glob(os.path.join(package_name, '*.py'))),
        (os.path.join('lib', package_name, 'nodes'), glob(os.path.join(package_name, 'nodes', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Autonomous Robotics Club of Purdue',
    maintainer_email='autonomy@purdue.edu',
    description='Handles localized actions to plan and follow trajectories.',
    license='BSD 3 Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bezier_path_server = rktl_planner.nodes.bezier_path_server:main',
            'path_follower = rktl_planner.nodes.path_follower:main',
            'path_planner = rktl_planner.nodes.path_planner:main',
            'patrol_planner = rktl_planner.nodes.patrol_planner:main'
        ],
    },
)
