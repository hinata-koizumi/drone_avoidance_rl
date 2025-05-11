# ────────────────────────────────────────────
# src/command_bridge/setup.py   ★NEW
# ────────────────────────────────────────────
from setuptools import setup

package_name = 'command_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=['command_bridge'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='user@example.com',
    description='DroneControlCommand → px4_msgs/ActuatorMotors bridge',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'command_bridge = command_bridge.main:main',
        ],
    },
    data_files=[
        (f'lib/{package_name}', ['command_bridge/main.py']),
    ],
)
