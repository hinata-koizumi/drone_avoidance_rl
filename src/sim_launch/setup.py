import os
from glob import glob

from setuptools import setup

package_name = 'sim_launch'

setup(
    name=package_name,
    version='2.0.1',
    packages=[],
    include_package_data=True,

    entry_points={
        'console_scripts': [],
    },

    data_files=[
         (f'share/{package_name}', ['package.xml']),
         (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
         (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
          [f'resource/{package_name}']),
     ],

    install_requires=[
        'setuptools',
        'rclpy',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Launch files to run Gazebo and bridge nodes.',
    license='MIT',
)
