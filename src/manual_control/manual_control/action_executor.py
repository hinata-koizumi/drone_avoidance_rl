#!/usr/bin/env python3

import os
import yaml
import time
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, MagneticField, FluidPressure
from std_msgs.msg import Header

from ament_index_python.packages import get_package_share_directory


class ActionType(Enum):
    """事前定義された行動タイプ"""
    HOVER = "hover"
    TAKEOFF = "takeoff"
    LANDING = "landing"
    WAYPOINT = "waypoint"
    CIRCLE = "circle"
    SQUARE = "square"
    MANUAL = "manual"
    RESET = "reset"


@dataclass
class ActionSequence:
    """行動シーケンス定義"""
    name: str
    action_type: ActionType
    duration: float
    parameters: Dict[str, Any]
    next_action: Optional[str] = None


class ActionExecutorNode(Node):
    """
    事前定義された行動をドローンに実行するノード
    """
    
    def __init__(self) -> None:
        """初期化"""
        super().__init__('action_executor_node')
        
        # QoS設定
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # パブリッシャー・サブスクライバー設定
        self._setup_communication()
        
        # 行動シーケンス読み込み
        self.action_sequences = self._load_action_sequences()
        
        # 追加: ドローン仕様読み込み
        self.drone_specs = self._load_drone_specs()
        if self.drone_specs:
            self.get_logger().info("Loaded drone_specs.yaml successfully")
        else:
            self.get_logger().warn("Failed to load drone_specs.yaml or file is empty")
        
        # 現在の状態
        self.current_action: Optional[ActionSequence] = None
        self.action_start_time: Optional[float] = None
        self.drone_state: Optional[TwistStamped] = None
        
        # タイマー設定
        self.timer = self.create_timer(0.1, self._execute_action)  # 10Hz
        
        self.get_logger().info("ActionExecutorNode initialized")
    
    def _setup_communication(self) -> None:
        """通信設定"""
        # 制御コマンドパブリッシャー
        self.control_pub = self.create_publisher(
            TwistStamped,
            '/drone/control_command',
            self.qos_profile
        )
        
        # ドローン状態サブスクライバー
        self.state_sub = self.create_subscription(
            TwistStamped,  # 標準的なメッセージ型
            '/drone/state',
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
            'action_sequences.yaml'
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
    
    def _load_drone_specs(self) -> dict:
        """ドローン仕様ファイルを読み込み"""
        specs_path = os.path.join(
            get_package_share_directory('manual_control'),
            'config',
            'drone_specs.yaml'
        )
        try:
            with open(specs_path, 'r') as f:
                data = yaml.safe_load(f)
            return data.get('drone_specifications', {})
        except Exception as e:
            self.get_logger().error(f"Failed to load drone_specs.yaml: {e}")
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
    
    def reset_drone(self) -> None:
        """ドローンをリセット（位置のみ）"""
        # 現在の行動を停止
        self.current_action = None
        self.action_start_time = None
        
        # リセットコマンドを送信（位置のみリセット）
        command = TwistStamped()
        command.header = Header()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "drone_base_link"
        
        # 位置をリセットするための特別なコマンドとして、すべての値を0に送信
        command.twist.linear.x = 0.0
        command.twist.linear.y = 0.0
        command.twist.linear.z = 0.0
        command.twist.angular.x = 0.0
        command.twist.angular.y = 0.0
        command.twist.angular.z = 0.0
        
        self.control_pub.publish(command)
        self.get_logger().info("Reset command sent to drone (position only)")
    
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
    
    def _generate_command(self) -> Optional[TwistStamped]:
        """現在の行動に基づいて制御コマンドを生成"""
        if not self.current_action:
            return None
        
        command = TwistStamped()
        command.header = Header()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "drone_base_link"
        
        if self.current_action.action_type == ActionType.HOVER:
            # ホバリング制御
            command.twist.linear.x = 0.0
            command.twist.linear.y = 0.0
            command.twist.linear.z = 0.0
            command.twist.angular.x = 0.0
            command.twist.angular.y = 0.0
            command.twist.angular.z = 0.0
            
        elif self.current_action.action_type == ActionType.TAKEOFF:
            # 離陸制御
            command.twist.linear.x = 0.0
            command.twist.linear.y = 0.0
            command.twist.linear.z = 0.7
            command.twist.angular.x = 0.0
            command.twist.angular.y = 0.0
            command.twist.angular.z = 0.0
            
        elif self.current_action.action_type == ActionType.LANDING:
            # 着陸制御
            command.twist.linear.x = 0.0
            command.twist.linear.y = 0.0
            command.twist.linear.z = 0.3
            command.twist.angular.x = 0.0
            command.twist.angular.y = 0.0
            command.twist.angular.z = 0.0
            
        elif self.current_action.action_type == ActionType.WAYPOINT:
            # ウェイポイント移動
            target_x = self.current_action.parameters.get('target_x', 0.0)
            target_y = self.current_action.parameters.get('target_y', 0.0)
            # 簡易的なPID制御（実際の実装ではより複雑）
            command.twist.linear.x = 0.5
            command.twist.linear.y = 0.5
            command.twist.linear.z = 0.0
            command.twist.angular.x = target_x * 0.1
            command.twist.angular.y = target_y * 0.1
            command.twist.angular.z = 0.0
            
        elif self.current_action.action_type == ActionType.CIRCLE:
            # 円形飛行
            radius = self.current_action.parameters.get('radius', 5.0)
            elapsed_time = time.time() - self.action_start_time
            duration = self.current_action.duration
            # 1周分の角速度を計算
            angular_velocity = 2 * 3.14159265 / duration  # [rad/s] 1周分
            angle = angular_velocity * elapsed_time
            # 円運動のX/Y成分を角度に反映（簡易的な制御）
            command.twist.linear.x = 0.5
            command.twist.linear.y = 0.5
            command.twist.linear.z = 0.0
            command.twist.angular.x = radius * 0.1 * float(np.cos(angle))
            command.twist.angular.y = radius * 0.1 * float(np.sin(angle))
            command.twist.angular.z = 0.0
            
        elif self.current_action.action_type == ActionType.SQUARE:
            # 四角形パターン飛行
            side_length = self.current_action.parameters.get('side_length', 5.0)
            elapsed_time = time.time() - self.action_start_time
            side_duration = self.current_action.duration / 4
            
            current_side = int(elapsed_time / side_duration) % 4
            side_progress = (elapsed_time % side_duration) / side_duration
            
            if current_side == 0:  # 前進
                command.twist.linear.x = side_length * 0.1 * side_progress
                command.twist.linear.y = 0.0
                command.twist.linear.z = 0.5
                command.twist.angular.x = 0.0
                command.twist.angular.y = 0.0
                command.twist.angular.z = 0.0
            elif current_side == 1:  # 右移動
                command.twist.linear.x = side_length * 0.1
                command.twist.linear.y = side_length * 0.1 * side_progress
                command.twist.linear.z = 0.5
                command.twist.angular.x = 0.0
                command.twist.angular.y = 0.0
                command.twist.angular.z = 0.0
            elif current_side == 2:  # 後退
                command.twist.linear.x = side_length * 0.1 * (1 - side_progress)
                command.twist.linear.y = side_length * 0.1
                command.twist.linear.z = 0.5
                command.twist.angular.x = 0.0
                command.twist.angular.y = 0.0
                command.twist.angular.z = 0.0
            else:  # 左移動
                command.twist.linear.x = 0.0
                command.twist.linear.y = side_length * 0.1 * (1 - side_progress)
                command.twist.linear.z = 0.5
                command.twist.angular.x = 0.0
                command.twist.angular.y = 0.0
                command.twist.angular.z = 0.0
                
        elif self.current_action.action_type == ActionType.RESET:
            # リセット制御
            command.twist.linear.x = 0.0
            command.twist.linear.y = 0.0
            command.twist.linear.z = 0.0
            command.twist.angular.x = 0.0
            command.twist.angular.y = 0.0
            command.twist.angular.z = 0.0
        
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
    
    def _state_callback(self, msg: TwistStamped) -> None:
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
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            executor.shutdown()
        except Exception as e:
            print(f"Warning: Error during cleanup: {e}")
        
        # rclpy.shutdown()は既に呼ばれている可能性があるため、try-exceptで囲む
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            # 既にシャットダウンされている場合は無視
            pass


if __name__ == '__main__':
    main() 