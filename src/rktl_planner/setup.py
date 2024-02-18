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
        ],
    },
)
