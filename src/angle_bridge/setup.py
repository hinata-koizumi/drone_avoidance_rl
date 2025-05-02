# ────────────────────────────────────────────
# src/angle_bridge/setup.py   ★FIX I-1
# ────────────────────────────────────────────
from setuptools import setup

package_name = "angle_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="user@example.com",
    description="Convert auxiliary-propeller angles to PX4 ActuatorServos",
    license="Apache 2.0",
    entry_points={
        "console_scripts": [
            "angle_bridge = angle_bridge.angle_bridge:main",
        ],
    },
)
