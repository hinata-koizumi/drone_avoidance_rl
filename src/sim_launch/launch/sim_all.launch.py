import os
import sys

from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration

WORLD_PATH = "/usr/share/gz/gz-sim7/worlds/empty.sdf"
if not os.path.exists(WORLD_PATH):
    raise RuntimeError(f"World file not found: {WORLD_PATH}")


def generate_launch_description() -> LaunchDescription:
    install_dir = os.environ.get('INSTALL_DIR', '/sim_ws/install')
    py_version = f"{sys.version_info.major}.{sys.version_info.minor}"
    pythonpath = os.pathsep.join([
        os.path.join(install_dir, f"px4_msgs/lib/python{py_version}/site-packages"),
        os.path.join(install_dir, f"drone_msgs/lib/python{py_version}/site-packages"),
        os.environ.get("PYTHONPATH", "")
    ])
    ld_library_path = os.pathsep.join([
        "/opt/ros/humble/lib",
        "/opt/ros/humble/local/lib",
        os.environ.get("LD_LIBRARY_PATH", "")
    ])
    base_env = os.environ.copy()
    base_env["PYTHONPATH"] = pythonpath
    base_env["LD_LIBRARY_PATH"] = ld_library_path
    base_env["RCUTILS_LOGGING_DIRECTORY"] = "/tmp/.ros/log"
    return LaunchDescription([
        # GUI 表示オプション
        DeclareLaunchArgument(
            "headless", default_value="true",
            description="Run simulation without GUI"
        ),

        # Gazebo Garden 起動（gz sim）
        ExecuteProcess(
            cmd=[
                "gz", "sim", "-r",
                WORLD_PATH,
                "--headless-rendering", "--verbose"
            ],
            output="screen",
            condition=UnlessCondition(LaunchConfiguration("headless"))
        ),
        ExecuteProcess(
            cmd=[
                "gz", "sim", "-r", "-s",
                WORLD_PATH,
                "--headless-rendering", "--verbose"
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("headless"))
        ),
        # PX4 SITL 起動
        ExecuteProcess(
            cmd=[
                "px4", "-i", "0", "-d",
                "-s", "/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/rcS",
                "-w", "build/px4_sitl_rtps"
            ],
            output="screen",
            env=base_env
        ),

        # 各ブリッジノード（ROS2 側）
        ExecuteProcess(
            cmd=["python3", os.path.join(install_dir, "command_bridge/lib/command_bridge/main.py")],
            output="screen",
            env=base_env
        ),
        ExecuteProcess(
            cmd=["python3", os.path.join(install_dir, "state_bridge/lib/state_bridge/state_bridge.py")],
            output="screen",
            env=base_env
        ),
        ExecuteProcess(
            cmd=["python3", os.path.join(install_dir, "angle_bridge/lib/angle_bridge/main.py")],
            output="screen",
            env=base_env
        ),
        ExecuteProcess(
            cmd=["python3", os.path.join(install_dir, "outer_motor_bridge/lib/outer_motor_bridge/main.py")],
            output="screen",
            env=base_env
        ),
    ])
