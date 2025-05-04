# src/state_bridge/setup.py  ← ★新規追加
from setuptools import setup, find_packages

package_name = "state_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),           # state_bridge/ 内を自動検出
    data_files=[                        # ★これが無いと ros2 pkg list に出ない
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "rclpy"],
    zip_safe=True,
    maintainer="Hinata Koizumi",
    maintainer_email="your@mail",
    description="Bridge drone state → RL agent",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "state_bridge_node = state_bridge.state_bridge:main",  # ← 実行エントリ
        ],
    },
)
