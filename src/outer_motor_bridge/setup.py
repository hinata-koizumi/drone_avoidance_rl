# src/outer_motor_bridge/setup.py
from setuptools import setup

setup(
    name="outer_motor_bridge",
    version="0.1.0",
    py_modules=["outer_motor_bridge"],   # ★ここ
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/outer_motor_bridge"]),
        ("share/outer_motor_bridge", ["package.xml"]),
    ],
    install_requires=["setuptools", "rclpy"],
    entry_points={
        "console_scripts": [
            "outer_motor_bridge_node = outer_motor_bridge:main",
        ],
    },
)
