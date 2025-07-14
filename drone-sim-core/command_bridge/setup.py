from setuptools import setup

package_name = 'command_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL',
    description='Command bridge node for drone avoidance RL',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_bridge = command_bridge.main:main'
        ],
    },
) 