from setuptools import setup

package_name = 'outer_motor_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=['outer_motor_bridge'],
    install_requires=['setuptools==59.6.0', 'rclpy==1.12.0'],
    zip_safe=True,
    maintainer='Hinata Koizumi',
    maintainer_email='example@example.com',
    description='PX4 fan PWM → Gazebo joint コマンド橋渡しノード',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'outer_motor_bridge_node = outer_motor_bridge.main:main',
        ],
    },
    data_files=[
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/resource', ['resource/outer_motor_bridge']),
    ],
)
