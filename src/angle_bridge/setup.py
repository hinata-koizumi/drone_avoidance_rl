from setuptools import setup, find_packages
setup(
    name='angle_bridge',
    version='0.1.0',
    packages=find_packages(),                # ← 固定文字列をやめる
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/angle_bridge']),
        ('share/angle_bridge', ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    entry_points={
        'console_scripts': ['angle_bridge_node = angle_bridge.main:main'],
    },
)
