from setuptools import setup

package_name = 'outer_motor_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=['outer_motor_bridge'],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='Hinata Koizumi',
    maintainer_email='example@example.com',
    description='PX4 fan PWM → Gazebo joint コマンド橋渡しノード',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'outer_motor_bridge = outer_motor_bridge.main:main',
        ],
    },
    data_files=[
        (f'lib/{package_name}', ['outer_motor_bridge/main.py']),
    ],
)
