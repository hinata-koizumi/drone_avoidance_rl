# type: ignore
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description() -> LaunchDescription:
    # YAMLからパラメータ読み込み
    config_path = os.path.join(get_package_share_directory('sim_launch'), 'config', 'sim_params.yaml')
    with open(config_path, 'r') as f:
        params = yaml.safe_load(f)

    # Declare launch arguments for key params
    launch_args = [
        DeclareLaunchArgument('cmd_topic', default_value=params['cmd_topic']),
        DeclareLaunchArgument('gz_world', default_value=params['gz_world']),
        DeclareLaunchArgument('physics_engine', default_value=params['physics_engine']),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo起動引数 - デフォルト値を直接指定
    gz_args_str = '-r ' + params['gz_world'] + ' --physics-engine ' + params['physics_engine']

    ign_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments=[
            ('gz_args', gz_args_str),
            ('use_sim_time', use_sim_time),
        ],
    )

    set_gz_env = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', '/usr/share/gz/garden/models:/root/.gz/models'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', '/usr/share/gz/garden/models:/root/.gz/models'),
    ]

    bridge_nodes = [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
            output='screen',
        ),
    ]
    # YAML bridge_topicsから追加ブリッジノードを生成
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
            continue
        bridge_nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[arg],
            output='screen',
        ))

    # === 可視化ツールの自動起動（CI環境では無効化） ===
    visualization_nodes = []
    # CI環境では可視化ツールを無効化
    # visualization_nodes = [
    #     Node(
    #         package='rqt_graph',
    #         executable='rqt_graph',
    #         name='rqt_graph',
    #         output='screen',
    #     ),
    #     Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         name='rviz2',
    #         output='screen',
    #         arguments=['-d', os.path.join(get_package_share_directory('sim_launch'), 'resource', 'default.rviz')]
    #         if os.path.exists(
    #             os.path.join(
    #                 get_package_share_directory('sim_launch'),
    #                 'resource',
    #                 'default.rviz'
    #             )
    #         ) else [],
    #     ),
    # ]

    return LaunchDescription([
        *launch_args,
        *set_gz_env,
        ign_gazebo_launch,
        *bridge_nodes,
        *visualization_nodes,
    ]) 