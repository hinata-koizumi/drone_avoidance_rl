# type: ignore
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # パラメータ取得
    use_sim_time = os.environ.get('USE_SIM_TIME', 'true').lower() == 'true'
    gz_world = os.environ.get('IGN_GAZEBO_WORLD', 'empty.sdf')
    headless = os.environ.get('GAZEBO_HEADLESS', 'true').lower() == 'true'

    # Gazebo起動引数
    gz_args = f'-r {gz_world} --headless-rendering' if headless else f'-r {gz_world}'

    # Ignition Gazebo起動
    ign_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments=[
            ('gz_args', gz_args),
            ('use_sim_time', str(use_sim_time).lower()),
        ],
    )

    # ブリッジノード群
    bridge_nodes = [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
            output='screen',
        ),
        # 必要に応じて他のブリッジも追加
    ]

    return LaunchDescription([
        ign_gazebo_launch,
        *bridge_nodes,
    ]) 