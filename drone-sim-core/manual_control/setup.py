from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'manual_control'

# 設定ファイルのパスを絶対パスで指定
config_files = []
config_dir = os.path.join(os.path.dirname(__file__), '../../config')
if os.path.exists(config_dir):
    config_files = glob(os.path.join(config_dir, '*.yaml'))

# launchファイルのパスを絶対パスで指定
launch_files = []
launch_dir = os.path.join(os.path.dirname(__file__), 'launch')
if os.path.exists(launch_dir):
    launch_files = glob(os.path.join(launch_dir, '*.py'))

setup(
    name=package_name,
    version='2.0.1',
    packages=find_packages(),
    python_requires='>=3.8',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), config_files),
        (os.path.join('share', package_name, 'launch'), launch_files),
    ],
    install_requires=[
        'setuptools',
        'pyyaml',
        'numpy',
    ],
    zip_safe=True,
    maintainer='drone_developer',
    maintainer_email='developer@example.com',
    description='Manual control system for predefined drone actions',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_executor = manual_control.action_executor:main',
            'simple_simulator = manual_control.simple_simulator:main',
            'state_monitor = manual_control.state_monitor:main',
        ],
    },
) 