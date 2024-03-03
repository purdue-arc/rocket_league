from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'rktl_dependencies'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), glob(os.path.join(package_name, '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Schmok',
    maintainer_email='aschmok@purdue.edu',
    description='Contains external dependencies required for other packages.',
    license='BSD 3 Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
