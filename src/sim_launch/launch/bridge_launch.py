# type: ignore
import yaml
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    config_path = os.path.join(get_package_share_directory('sim_launch'), 'config', 'sim_params.yaml')
    with open(config_path, 'r') as f:
        params = yaml.safe_load(f)
    launch_args = []
    bridge_nodes = []
    for bridge in params.get('bridge_topics', []):
        topic = bridge['topic']
        ros_type = bridge['ros_type']
        ign_type = bridge['ign_type']
        direction = bridge.get('direction', 'bidirectional')
        if direction == 'bidirectional':
            arg = f"{topic}@{ros_type}[{ign_type}"
        elif direction == 'ros_to_ign':
            arg = f"{topic}@{ros_type}]<{ign_type}"
        elif direction == 'ign_to_ros':
            arg = f"{topic}@{ros_type}[>{ign_type}"
        else:
            continue  # skip unknown direction
        bridge_nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[arg],
            output='screen',
        ))
    return LaunchDescription([
        *launch_args,
        *bridge_nodes,
    ])
