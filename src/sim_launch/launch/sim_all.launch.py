import os
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    install_dir = os.environ.get('INSTALL_DIR', '/sim_ws/install')
    pythonpath = os.pathsep.join([
        os.path.join(install_dir, "px4_msgs/lib/python3.10/site-packages"),
        os.path.join(install_dir, "drone_msgs/lib/python3.10/site-packages"),
        os.environ.get("PYTHONPATH", "")
    ])
    ld_library_path = os.pathsep.join([
        "/opt/ros/humble/lib",
        "/opt/ros/humble/local/lib",
        os.environ.get("LD_LIBRARY_PATH", "")
    ])
    return LaunchDescription([
        # GUI 表示オプション
        DeclareLaunchArgument(
            "headless", default_value="true",
            description="Run simulation without GUI"
        ),

        # Gazebo Garden 起動（gz sim）
        ExecuteProcess(
            cmd=["timeout", "60s", "gz", "sim", "-r", "/usr/share/gz/gz-sim7/worlds/empty.sdf"],
            output="screen",
            condition=UnlessCondition(LaunchConfiguration("headless"))
        ),
        ExecuteProcess(
            cmd=["timeout", "60s", "gz", "sim", "-r", "-s", "/usr/share/gz/gz-sim7/worlds/empty.sdf"],
            output="screen",
            condition=IfCondition(LaunchConfiguration("headless"))
        ),

        # 各ブリッジノード（ROS2 側）
        ExecuteProcess(
            cmd=["python3", os.path.join(install_dir, "command_bridge/lib/command_bridge/main.py")],
            output="screen",
            env={"PYTHONPATH": pythonpath, "LD_LIBRARY_PATH": ld_library_path}
        ),
        ExecuteProcess(
            cmd=["python3", os.path.join(install_dir, "state_bridge/lib/state_bridge/state_bridge.py")],
            output="screen",
            env={"PYTHONPATH": pythonpath, "LD_LIBRARY_PATH": ld_library_path}
        ),
        ExecuteProcess(
            cmd=["python3", os.path.join(install_dir, "angle_bridge/lib/angle_bridge/main.py")],
            output="screen",
            env={"PYTHONPATH": pythonpath, "LD_LIBRARY_PATH": ld_library_path}
        ),
        ExecuteProcess(
            cmd=["python3", os.path.join(install_dir, "outer_motor_bridge/lib/outer_motor_bridge/main.py")],
            output="screen",
            env={"PYTHONPATH": pythonpath, "LD_LIBRARY_PATH": ld_library_path}
        ),
    ])
