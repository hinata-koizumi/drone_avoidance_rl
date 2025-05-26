# type: ignore
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    # YAMLからパラメータ読み込み
    config_path = os.path.join(os.path.dirname(__file__), '../../config/sim_params.yaml')
    with open(config_path, 'r') as f:
        params = yaml.safe_load(f)

    # Declare launch arguments for key params
    launch_args = [
        DeclareLaunchArgument('cmd_topic', default_value=params['cmd_topic']),
        DeclareLaunchArgument('gz_world', default_value=params['gz_world']),
        DeclareLaunchArgument('physics_engine', default_value=params['physics_engine']),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gz_world = LaunchConfiguration('gz_world')
    physics_engine = LaunchConfiguration('physics_engine')
    headless = LaunchConfiguration('headless', default='true')
    cmd_topic = LaunchConfiguration('cmd_topic')

    # Gazebo起動引数
    gz_args = [
        '-r', gz_world,
        '--physics-engine', physics_engine,
    ]
    if headless.perform({}).lower() == 'true':
        gz_args.append('--headless-rendering')
    gz_args_str = ' '.join(gz_args)

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
        # 他のブリッジもYAMLからパラメータ化して追加可能
    ]

    return LaunchDescription([
        *launch_args,
        *set_gz_env,
        ign_gazebo_launch,
        *bridge_nodes,
    ]) 