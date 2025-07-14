from setuptools import find_packages, setup

# print('find_packages() result:', find_packages())

setup(
    name='state_bridge',
    version='0.1.0',
    packages=find_packages(exclude=['tests*']),
    install_requires=['setuptools==59.6.0', 'rclpy==1.12.0'],
    zip_safe=False,
    maintainer='Hinata Koizumi',
    maintainer_email='example@example.com',
    description='PX4 fan PWM → Gazebo joint コマンド橋渡しノード',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_bridge_node = state_bridge:main',
        ],
    },
    data_files=[
        ('share/state_bridge', ['package.xml']),
        ('share/state_bridge/resource', ['resource/state_bridge']),
    ],
)
