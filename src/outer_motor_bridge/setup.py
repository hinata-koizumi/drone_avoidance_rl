from setuptools import setup
package_name = "outer_motor_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="user@example.com",
    description="Republish PX4 ActuatorMotors (outer rotors) for logging",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "outer_motor_bridge = outer_motor_bridge.outer_motor_bridge:main",
        ],
    },
)
