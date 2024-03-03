import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rktl_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DinoSage',
    maintainer_email='anshag@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "controller = rktl_control.controller:main",
            "mean_odom_filter = rktl_control.mean_odom_filter:main",
            "particle_odom_filter = rktl_control.particle_odom_filter:main",
            "topic_delay = rktl_control.topic_delay:main",
            "keyboard_interface = rktl_control.keyboard_interface:main",
            "pose_synchronizer.py = rktl_control.pose_synchronizer:main"
        ],
    },
)
