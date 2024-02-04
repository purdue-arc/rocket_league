from setuptools import find_packages, setup

package_name = 'rktl_launch'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Autonomous Robotics Club of Purdue',
    maintainer_email='autonomy@purdue.edu',
    description='Launch files for the ARC Rocket League project',
    license='BSD 3 Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)