from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'assignment1_rt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hani',
    maintainer_email='hani@todo.todo',
    description='Assignment 1 RT',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ui_node = assignment1_rt.ui_node:main',
            'distance_node = assignment1_rt.distance_node:main',
            'spawner = assignment1_rt.turtle_spawn:main',
        ],
    },
)
