from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
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
            cmd=["python3", "/sim_ws/install/command_bridge/lib/command_bridge/main.py"],
            output="screen"
        ),
        ExecuteProcess(
            cmd=["python3", "/sim_ws/install/state_bridge/lib/state_bridge/state_bridge.py"],
            output="screen"
        ),
        ExecuteProcess(
            cmd=["python3", "/sim_ws/install/angle_bridge/lib/angle_bridge/main.py"],
            output="screen"
        ),
        ExecuteProcess(
            cmd=["python3", "/sim_ws/install/outer_motor_bridge/lib/outer_motor_bridge/main.py"],
            output="screen"
        ),
    ])
