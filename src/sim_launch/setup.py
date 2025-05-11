import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'sim_launch'

setup(
    name=package_name,
    version='0.0.1',
    # Python モジュールを自動で検出
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    include_package_data=True,

    entry_points={
        'console_scripts': [
            # "ros2 run sim_launch e2e_test" で呼べるようになります
            'e2e_test = sim_launch.e2e_test:main',
        ],
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
