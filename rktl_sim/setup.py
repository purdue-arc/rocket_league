from setuptools import find_packages, setup

package_name = 'rktl_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Autonomous Robotics Club of Purdue',
    maintainer_email='autonomy@purdue.eduo',
    description='Simulator and Visualizer for ARC Rocket League project',
    license='BSD 3 Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
