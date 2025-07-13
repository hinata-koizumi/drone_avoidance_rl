#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import time
from typing import Dict, Any
import yaml

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Header


class OptimizedDroneSimulator(Node):
    """
    最適化されたドローンのシミュレーター
    高性能な物理計算と効率的な制御システム
    """
    
    def __init__(self):
        super().__init__('optimized_drone_simulator')
        
        # 設定ファイルの読み込み
        self.drone_config = self._load_drone_config()
        
        # QoS設定（最適化）
        self.qos_profile = QoSProfile(
            depth=5,  # バッファサイズを削減
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 高速化
            history=HistoryPolicy.KEEP_LAST
        )
        
        # ドローンの状態（NumPy配列で高速化）
        self.state = np.zeros(12)  # [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
        
        # 物理パラメータ（設定から取得）
        self.mass = self.drone_config['physical']['weight']
        self.gravity = 9.81
        self.thrust_constant = self.drone_config['motors']['main_rotors']['thrust_constant']
        self.max_thrust = self._calculate_max_thrust()
        
        # 制御パラメータ（最適化されたPID）
        self.attitude_gains = np.array([
            self.drone_config['control']['attitude']['roll_p'],
            self.drone_config['control']['attitude']['pitch_p'],
            self.drone_config['control']['attitude']['yaw_p']
        ])
        
        # 性能制限
        self.max_velocity = self.drone_config['performance']['max_speed_horizontal']
        self.max_acceleration = self.drone_config['performance']['max_acceleration']
        
        # 制御入力
        self.control_input = np.zeros(4)  # [throttle, roll, pitch, yaw]
        
        # 通信設定
        self._setup_communication()
        
        # 高頻度シミュレーション (200Hz)
        self.timer = self.create_timer(0.005, self._simulation_step)
        
        # パフォーマンス計測
        self.step_count = 0
        self.start_time = time.time()
        
        self.get_logger().info("Optimized Drone Simulator initialized")
    
    def _load_drone_config(self) -> Dict[str, Any]:
        """ドローン設定の読み込み"""
        try:
            with open('/workspace/config/drone_specs.yaml', 'r') as f:
                config = yaml.safe_load(f)
            return config['drone_specifications']
        except Exception as e:
            self.get_logger().warn(f"Failed to load config: {e}, using defaults")
            return self._get_default_config()
    
    def _get_default_config(self) -> Dict[str, Any]:
        """デフォルト設定"""
        return {
            'physical': {'weight': 0.65},
            'motors': {'main_rotors': {'thrust_constant': 1.6}},
            'control': {
                'attitude': {'roll_p': 4.8, 'pitch_p': 4.8, 'yaw_p': 3.2}
            },
            'performance': {'max_speed_horizontal': 23.6, 'max_acceleration': 12.0}
        }
    
    def _calculate_max_thrust(self) -> float:
        """最大推力の計算"""
        motor_count = self.drone_config['motors']['main_rotors']['count']
        max_rpm = self.drone_config['motors']['main_rotors']['max_rpm']
        # 簡易的な推力計算
        return motor_count * self.thrust_constant * (max_rpm / 10000.0)
    
    def _setup_communication(self):
        """通信設定（最適化）"""
        # 制御コマンドサブスクライバー
        self.control_sub = self.create_subscription(
            TwistStamped,
            '/drone/control_command',
            self._control_callback,
            self.qos_profile
        )
        
        # 位置・姿勢パブリッシャー
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/drone/pose',
            self.qos_profile
        )
        
        # 速度パブリッシャー
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/drone/twist',
            self.qos_profile
        )
        
        # IMUパブリッシャー
        self.imu_pub = self.create_publisher(
            Imu,
            '/imu/data',
            self.qos_profile
        )
    
    def _control_callback(self, msg: TwistStamped):
        """制御コマンドコールバック（最適化）"""
        # 制御入力を更新
        self.control_input[0] = msg.twist.linear.z  # throttle
        self.control_input[1] = msg.twist.angular.x  # roll
        self.control_input[2] = msg.twist.angular.y  # pitch
        self.control_input[3] = msg.twist.angular.z  # yaw
        
        # 制御入力の制限（throttleは負の値を許可、その他は-1.0から1.0）
        self.control_input[0] = np.clip(self.control_input[0], -1.0, 1.0)  # throttle
        self.control_input[1:4] = np.clip(self.control_input[1:4], -1.0, 1.0)  # roll, pitch, yaw
    
    def _simulation_step(self):
        """シミュレーションステップ（最適化）"""
        dt = 0.005  # 5ms
        
        # 状態更新
        self._update_dynamics(dt)
        
        # 物理制約の適用
        self._apply_constraints()
        
        # メッセージのパブリッシュ
        self._publish_messages()
        
        # パフォーマンス計測
        self.step_count += 1
        if self.step_count % 200 == 0:  # 1秒に1回
            elapsed = time.time() - self.start_time
            fps = self.step_count / elapsed
            self.get_logger().info(f"Simulation FPS: {fps:.1f}")
    
    def _update_dynamics(self, dt: float):
        """動力学の更新（最適化）"""
        # 推力計算
        throttle = self.control_input[0]  # abs()を削除して負の値を許可
        thrust = throttle * self.max_thrust
        
        # 角度制御（最適化されたPID）
        attitude_errors = self.control_input[1:4] - self.state[6:9]
        angular_acceleration = self.attitude_gains * attitude_errors
        
        # 角速度更新
        self.state[9:12] += angular_acceleration * dt
        
        # 姿勢更新
        self.state[6:9] += self.state[9:12] * dt
        
        # 推力ベクトル計算（最適化）
        roll, pitch = self.state[6], self.state[7]
        thrust_vector = np.array([
            thrust * np.sin(pitch),
            -thrust * np.sin(roll),
            thrust * np.cos(roll) * np.cos(pitch)
        ])
        
        # 加速度計算
        acceleration = thrust_vector / self.mass - np.array([0.0, 0.0, self.gravity])
        
        # 速度更新
        self.state[3:6] += acceleration * dt
        
        # 位置更新
        self.state[0:3] += self.state[3:6] * dt
    
    def _apply_constraints(self):
        """物理制約の適用"""
        # 地面との衝突
        if self.state[2] < 0.0:
            self.state[2] = 0.0
            self.state[5] = 0.0
        
        # 速度制限
        velocity_magnitude = np.linalg.norm(self.state[3:6])
        if velocity_magnitude > self.max_velocity:
            self.state[3:6] *= self.max_velocity / velocity_magnitude
        
        # 加速度制限
        acceleration = np.linalg.norm(self.state[9:12])
        if acceleration > self.max_acceleration:
            self.state[9:12] *= self.max_acceleration / acceleration
    
    def _publish_messages(self):
        """メッセージのパブリッシュ（最適化）"""
        current_time = self.get_clock().now().to_msg()
        
        # 位置・姿勢メッセージ
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = self.state[0]
        pose_msg.pose.position.y = self.state[1]
        pose_msg.pose.position.z = self.state[2]
        
        # クォータニオン計算（最適化）
        pose_msg.pose.orientation = self._euler_to_quaternion(
            self.state[6], self.state[7], self.state[8]
        )
        
        self.pose_pub.publish(pose_msg)
        
        # 速度メッセージ
        twist_msg = TwistStamped()
        twist_msg.header.stamp = current_time
        twist_msg.header.frame_id = "world"
        twist_msg.twist.linear.x = self.state[3]
        twist_msg.twist.linear.y = self.state[4]
        twist_msg.twist.linear.z = self.state[5]
        twist_msg.twist.angular.x = self.state[9]
        twist_msg.twist.angular.y = self.state[10]
        twist_msg.twist.angular.z = self.state[11]
        
        self.twist_pub.publish(twist_msg)
        
        # IMUメッセージ
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = "imu_link"
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = -self.gravity
        imu_msg.angular_velocity.x = self.state[9]
        imu_msg.angular_velocity.y = self.state[10]
        imu_msg.angular_velocity.z = self.state[11]
        
        self.imu_pub.publish(imu_msg)
    
    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float):
        """オイラー角からクォータニオンへの変換（最適化）"""
        from geometry_msgs.msg import Quaternion
        import math
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        quat = Quaternion()
        quat.w = cy * cp * cr + sy * sp * sr
        quat.x = cy * cp * sr - sy * sp * cr
        quat.y = sy * cp * sr + cy * sp * cr
        quat.z = sy * cp * cr - cy * sp * sr
        
        return quat


def main():
    rclpy.init()
    node = OptimizedDroneSimulator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 