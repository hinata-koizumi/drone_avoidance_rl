from setuptools import setup
import os
from glob import glob

package_name = 'manual_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drone_developer',
    maintainer_email='developer@example.com',
    description='Manual control system for predefined drone actions',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_executor = manual_control.action_executor:main',
            'action_controller = manual_control.action_controller:main',
        ],
    },
) 