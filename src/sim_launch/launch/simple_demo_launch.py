#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """簡単なドローンシミュレーションデモのlaunchファイル"""
    
    # 引数定義
    drone_specs_file = LaunchConfiguration('drone_specs_file')
    action_sequence_file = LaunchConfiguration('action_sequence_file')
    
    return LaunchDescription([
        # 引数宣言
        DeclareLaunchArgument(
            'drone_specs_file',
            default_value='drone_specs.yaml',
            description='Drone specifications file'
        ),
        DeclareLaunchArgument(
            'action_sequence_file',
            default_value='action_sequences.yaml',
            description='Action sequence file'
        ),
        
        # シンプルドローンのシミュレーター
        Node(
            package='manual_control',
            executable='simple_simulator',
            name='simple_drone_simulator',
            output='screen',
            parameters=[{
                'mass': 0.65,
                'gravity': 9.81,
                'thrust_constant': 1.6,
                'max_thrust': 15.0,
                'roll_p': 4.8,
                'pitch_p': 4.8,
                'yaw_p': 3.2
            }]
        ),
        
        # アクション実行ノード
        Node(
            package='manual_control',
            executable='action_executor',
            name='action_executor_node',
            output='screen',
            parameters=[{
                'control_topic': '/drone/control_command',
                'state_topic': '/drone/pose',
                'action_sequence_file': action_sequence_file,
                'drone_specs_file': drone_specs_file,
                'qos_depth': 10,
                'qos_reliability': 'reliable',
                'qos_history': 'keep_last',
                'log_level': 'info'
            }]
        ),
        
        # 状態表示ノード（オプション）
        Node(
            package='manual_control',
            executable='state_monitor',
            name='state_monitor_node',
            output='screen',
            parameters=[{
                'pose_topic': '/drone/pose',
                'twist_topic': '/drone/twist',
                'imu_topic': '/imu/data',
                'update_rate': 1.0  # 1Hz
            }]
        )
    ]) 