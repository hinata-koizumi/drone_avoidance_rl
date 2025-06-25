#!/usr/bin/env python3

import os
import yaml
import time
from typing import Dict, List, Any
from dataclasses import dataclass
from enum import Enum
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor

from drone_msgs.msg import DroneControlCommand
from px4_msgs.msg import ActuatorMotors
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, MagneticField, FluidPressure

from common.bridge_base import BridgeBase


class ActionType(Enum):
    """事前定義された行動タイプ"""
    HOVER = "hover"
    TAKEOFF = "takeoff"
    LANDING = "landing"
    WAYPOINT = "waypoint"
    CIRCLE = "circle"
    SQUARE = "square"
    MANUAL = "manual"


@dataclass
class ActionSequence:
    """行動シーケンス定義"""
    name: str
    action_type: ActionType
    duration: float
    parameters: Dict[str, Any]
    next_action: str = None


class ActionExecutorNode(BridgeBase):
    """
    事前定義された行動をドローンに実行するノード
    """
    
    def __init__(self) -> None:
        """初期化"""
        super().__init__('action_executor_node', {
            'control_topic': '/drone/control_command',
            'state_topic': '/drone/state',
            'action_sequence_file': 'action_sequences.yaml',
            'qos_depth': 10,
            'qos_reliability': 'reliable',
            'qos_history': 'keep_last',
            'log_level': 'info'
        })
        
        # QoS設定
        self.qos_profile = QoSProfile(
            depth=self.get_parameter('qos_depth').value,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # パブリッシャー・サブスクライバー設定
        self._setup_communication()
        
        # 行動シーケンス読み込み
        self.action_sequences = self._load_action_sequences()
        
        # 現在の状態
        self.current_action = None
        self.action_start_time = None
        self.drone_state = None
        
        # タイマー設定
        self.timer = self.create_timer(0.1, self._execute_action)  # 10Hz
        
        self.get_logger().info("ActionExecutorNode initialized")
    
    def _setup_communication(self) -> None:
        """通信設定"""
        # 制御コマンドパブリッシャー
        control_topic = self.get_parameter('control_topic').value
        self.control_pub = self.create_publisher(
            DroneControlCommand,
            control_topic,
            self.qos_profile
        )
        
        # ドローン状態サブスクライバー
        state_topic = self.get_parameter('state_topic').value
        self.state_sub = self.create_subscription(
            DroneControlCommand,  # 仮のメッセージ型
            state_topic,
            self._state_callback,
            self.qos_profile
        )
        
        # センサーデータサブスクライバー
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self._imu_callback,
            self.qos_profile
        )
    
    def _load_action_sequences(self) -> Dict[str, ActionSequence]:
        """行動シーケンスファイルを読み込み"""
        config_path = os.path.join(
            get_package_share_directory('manual_control'),
            'config',
            self.get_parameter('action_sequence_file').value
        )
        
        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
            
            sequences = {}
            for seq_data in data.get('action_sequences', []):
                seq = ActionSequence(
                    name=seq_data['name'],
                    action_type=ActionType(seq_data['action_type']),
                    duration=seq_data['duration'],
                    parameters=seq_data.get('parameters', {}),
                    next_action=seq_data.get('next_action')
                )
                sequences[seq.name] = seq
            
            self.get_logger().info(f"Loaded {len(sequences)} action sequences")
            return sequences
            
        except Exception as e:
            self.get_logger().error(f"Failed to load action sequences: {e}")
            return {}
    
    def start_action(self, action_name: str) -> bool:
        """行動開始"""
        if action_name not in self.action_sequences:
            self.get_logger().error(f"Action '{action_name}' not found")
            return False
        
        self.current_action = self.action_sequences[action_name]
        self.action_start_time = time.time()
        self.get_logger().info(f"Started action: {action_name}")
        return True
    
    def _execute_action(self) -> None:
        """行動実行（タイマーコールバック）"""
        if self.current_action is None:
            return
        
        elapsed_time = time.time() - self.action_start_time
        
        if elapsed_time >= self.current_action.duration:
            # 行動完了
            self._complete_action()
            return
        
        # 行動実行
        command = self._generate_command()
        if command:
            self.control_pub.publish(command)
    
    def _generate_command(self) -> DroneControlCommand:
        """現在の行動に基づいて制御コマンドを生成"""
        if not self.current_action:
            return None
        
        command = DroneControlCommand()
        
        if self.current_action.action_type == ActionType.HOVER:
            # ホバリング制御
            command.throttle1 = 0.5  # 中立位置
            command.throttle2 = 0.5
            command.angle1 = 0.0     # 水平維持
            command.angle2 = 0.0
            
        elif self.current_action.action_type == ActionType.TAKEOFF:
            # 離陸制御
            command.throttle1 = 0.7  # 上昇推力
            command.throttle2 = 0.7
            command.angle1 = 0.0
            command.angle2 = 0.0
            
        elif self.current_action.action_type == ActionType.LANDING:
            # 着陸制御
            command.throttle1 = 0.3  # 下降推力
            command.throttle2 = 0.3
            command.angle1 = 0.0
            command.angle2 = 0.0
            
        elif self.current_action.action_type == ActionType.WAYPOINT:
            # ウェイポイント移動
            target_x = self.current_action.parameters.get('target_x', 0.0)
            target_y = self.current_action.parameters.get('target_y', 0.0)
            # 簡易的なPID制御（実際の実装ではより複雑）
            command.throttle1 = 0.5
            command.throttle2 = 0.5
            command.angle1 = target_x * 0.1  # 簡易的な角度制御
            command.angle2 = target_y * 0.1
            
        elif self.current_action.action_type == ActionType.CIRCLE:
            # 円形飛行
            radius = self.current_action.parameters.get('radius', 5.0)
            elapsed_time = time.time() - self.action_start_time
            duration = self.current_action.duration
            # 1周分の角速度を計算
            angular_velocity = 2 * 3.14159265 / duration  # [rad/s] 1周分
            angle = angular_velocity * elapsed_time
            # 円運動のX/Y成分を角度に反映（簡易的な制御）
            command.throttle1 = 0.5
            command.throttle2 = 0.5
            command.angle1 = radius * 0.1 * float(np.cos(angle))
            command.angle2 = radius * 0.1 * float(np.sin(angle))
            
        elif self.current_action.action_type == ActionType.SQUARE:
            # 四角形パターン飛行
            side_length = self.current_action.parameters.get('side_length', 5.0)
            elapsed_time = time.time() - self.action_start_time
            side_duration = self.current_action.duration / 4
            
            current_side = int(elapsed_time / side_duration) % 4
            side_progress = (elapsed_time % side_duration) / side_duration
            
            if current_side == 0:  # 前進
                command.angle1 = side_length * 0.1 * side_progress
                command.angle2 = 0.0
            elif current_side == 1:  # 右移動
                command.angle1 = side_length * 0.1
                command.angle2 = side_length * 0.1 * side_progress
            elif current_side == 2:  # 後退
                command.angle1 = side_length * 0.1 * (1 - side_progress)
                command.angle2 = side_length * 0.1
            else:  # 左移動
                command.angle1 = 0.0
                command.angle2 = side_length * 0.1 * (1 - side_progress)
            
            command.throttle1 = 0.5
            command.throttle2 = 0.5
        
        return command
    
    def _complete_action(self) -> None:
        """行動完了処理"""
        if not self.current_action:
            return
        
        self.get_logger().info(f"Completed action: {self.current_action.name}")
        
        # 次の行動があれば開始
        if self.current_action.next_action:
            self.start_action(self.current_action.next_action)
        else:
            # 行動終了
            self.current_action = None
            self.action_start_time = None
    
    def _state_callback(self, msg: DroneControlCommand) -> None:
        """ドローン状態コールバック"""
        self.drone_state = msg
    
    def _imu_callback(self, msg: Imu) -> None:
        """IMUデータコールバック"""
        # IMUデータを処理（必要に応じて）
        pass


def main() -> None:
    """メイン関数"""
    rclpy.init()
    node = ActionExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # デフォルトでホバリング開始
        node.start_action("hover")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main() 