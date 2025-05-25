from setuptools import setup

package_name = 'sim_launch'

setup(
    name=package_name,
    version='2.0.1',
    packages=[],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            [
                'launch/gz_sim.launch.py',
                'launch/bridge_launch.py',
                'launch/sim_all.launch.py',
            ],
        ),
        (
            'share/' + package_name + '/resource',
            ['resource/empty_custom.sdf'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Launch files to run Gazebo and bridge nodes.',
    license='MIT',
    tests_require=['pytest'],
) 