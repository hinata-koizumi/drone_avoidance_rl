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
            cmd=["gz", "sim", "-r", "worlds/your_world.sdf"],
            output="screen",
            condition=UnlessCondition(LaunchConfiguration("headless"))
        ),
        ExecuteProcess(
            cmd=["gz", "sim", "-r", "-s", "worlds/your_world.sdf"],
            output="screen",
            condition=IfCondition(LaunchConfiguration("headless"))
        ),

        # 各ブリッジノード（ROS2 側）
        Node(
            package="command_bridge",
            executable="command_bridge_node",
            name="command_bridge",
            output="screen"
        ),
        Node(
            package="state_bridge",
            executable="state_bridge_node",
            name="state_bridge",
            output="screen"
        ),
        Node(
            package="angle_bridge",
            executable="angle_bridge_node",
            name="angle_bridge",
            output="screen"
        ),
        Node(
            package="outer_motor_bridge",
            executable="outer_motor_bridge_node",
            name="outer_motor_bridge",
            output="screen"
        ),
    ])
