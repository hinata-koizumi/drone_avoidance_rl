from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> None:
    return LaunchDescription([
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
            output='screen'
        )
    ])
