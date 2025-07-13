#!/usr/bin/env python3

import time

from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


class SimpleDroneSimulator(Node):
    """
    シンプルなドローンのシミュレーター
    物理的な制約や重力を考慮した簡単なシミュレーション
    """
    
    def __init__(self):
        """初期化"""
        super().__init__('simple_drone_simulator')
        
        # 物理パラメータ
        self.mass = 1.0  # kg
        self.gravity = 9.81  # m/s^2
        self.max_thrust = 20.0  # N
        
        # 制御パラメータ
        self.roll_p = 2.0
        self.pitch_p = 2.0
        self.yaw_p = 2.0
        
        # ホバリング制御用PIDパラメータ（より応答性を向上）
        self.altitude_p = 0.02  # 比例ゲインをさらに下げてオーバーシュートを防止
        self.altitude_i = 0.0005  # 積分ゲインをさらに下げて安定性を向上
        self.altitude_d = 0.15  # 微分ゲインを上げて過度な応答を抑制
        self.altitude_integral = 0.0
        self.altitude_error_prev = 0.0
        self.target_altitude = 0.0  # 目標高度
        self.hover_start_time = 0.0  # ホバリング開始時間
        
        # 水平移動コマンドのタイムアウト機能
        self.horizontal_command_timeout = 10.0  # 10.0秒でタイムアウト（非常に長くして継続的な動きを実現）
        self.last_horizontal_command_time = 0.0
        
        # ヨー回転コマンドのタイムアウト機能
        self.yaw_command_timeout = 10.0  # 10.0秒でタイムアウト（継続的な回転を実現）
        self.last_yaw_command_time = 0.0
        
        # 状態変数
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        
        # 制御コマンド
        self.control_command = np.array([0.0, 0.0, 0.0, 0.0])  # throttle, roll, pitch, yaw
        self.horizontal_velocity_x = 0.0
        self.horizontal_velocity_y = 0.0
        
        # ホバリング状態フラグ
        self.is_hovering = False
        
        # リセット関連
        self.reset_time = 0.0
        self.reset_cooldown = 1.0  # リセット後1秒間は制御コマンドを無視
        
        # QoS設定
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # 通信設定
        self._setup_communication()
        
        # タイマー設定 (100Hz)
        self.timer = self.create_timer(0.01, self._simulation_step)
        
        self.get_logger().info("Simple Drone Simulator initialized")
    
    def reset_drone(self):
        """ドローンを初期位置にリスポーン"""
        # 位置と速度をリセットして即座に(0,0,0)にリスポーン
        old_position = self.position.copy()
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])  # 速度もリセット
        
        # PID制御の状態もリセット
        self.target_altitude = 0.0
        self.altitude_integral = 0.0
        self.altitude_error_prev = 0.0
        
        # ホバリングフラグもリセット
        self.is_hovering = False
        
        # リセット時間を記録
        self.reset_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info(
            f"Drone respawned from [{old_position[0]:.2f}, {old_position[1]:.2f}, "
            f"{old_position[2]:.2f}] to [0.00, 0.00, 0.00]"
        )
        self.get_logger().info(f"Position array after reset: {self.position}")
        self.get_logger().info(f"Reset cooldown started at {self.reset_time:.2f}s")
        
        # 位置・速度を即時パブリッシュ
        self._publish_state()
        
        # 制御コマンドもリセット
        self.control_command = np.array([0.0, 0.0, 0.0, 0.0])
        self.horizontal_velocity_x = 0.0
        self.horizontal_velocity_y = 0.0
        self.last_horizontal_command_time = 0.0  # 水平移動コマンドの時刻もリセット
        self.last_yaw_command_time = 0.0  # ヨー回転コマンドの時刻もリセット
    
    def _publish_state(self):
        """現在の位置・速度を即時パブリッシュ"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = self.position[0]
        pose_msg.pose.position.y = self.position[1]
        pose_msg.pose.position.z = self.position[2]
        # クォータニオン計算（簡易）
        import math
        cy = math.cos(self.orientation[2] * 0.5)
        sy = math.sin(self.orientation[2] * 0.5)
        cp = math.cos(self.orientation[1] * 0.5)
        sp = math.sin(self.orientation[1] * 0.5)
        cr = math.cos(self.orientation[0] * 0.5)
        sr = math.sin(self.orientation[0] * 0.5)
        pose_msg.pose.orientation.w = cy * cp * cr + sy * sp * sr
        pose_msg.pose.orientation.x = cy * cp * sr - sy * sp * cr
        pose_msg.pose.orientation.y = sy * cp * sr + cy * sp * cr
        pose_msg.pose.orientation.z = sy * cp * cr - cy * sp * sr
        self.pose_pub.publish(pose_msg)
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "world"
        twist_msg.twist.linear.x = self.velocity[0]
        twist_msg.twist.linear.y = self.velocity[1]
        twist_msg.twist.linear.z = self.velocity[2]
        twist_msg.twist.angular.x = self.angular_velocity[0]
        twist_msg.twist.angular.y = self.angular_velocity[1]
        twist_msg.twist.angular.z = self.angular_velocity[2]
        self.twist_pub.publish(twist_msg)
    
    def _setup_communication(self):
        """通信設定"""
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
            '/drone/imu',
            self.qos_profile
        )
        
        # 推力情報を追加
        self.thrust_pub = self.create_publisher(Float32, '/drone/thrust', 10)
    
    def _control_callback(self, msg: TwistStamped):
        """制御コマンドコールバック"""
        # リセット後のクールダウン期間中は制御コマンドを無視
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.reset_time < self.reset_cooldown:
            self.get_logger().debug(
                f"Ignoring control command during reset cooldown "
                f"({current_time - self.reset_time:.2f}s remaining)"
            )
            return
        
        # 制御コマンドを保存
        self.control_command[0] = msg.twist.linear.z  # throttle (Z軸)
        self.control_command[1] = msg.twist.angular.x  # roll
        self.control_command[2] = msg.twist.angular.y  # pitch
        self.control_command[3] = msg.twist.angular.z  # yaw
        
        # 水平移動コマンドを直接速度に変換
        self.horizontal_velocity_x = msg.twist.linear.x  # X軸の速度
        self.horizontal_velocity_y = msg.twist.linear.y  # Y軸の速度
        
        # 水平移動コマンドが受信された時刻を記録
        if abs(msg.twist.linear.x) > 0.01 or abs(msg.twist.linear.y) > 0.01:
            self.last_horizontal_command_time = current_time
        
        # ヨー回転コマンドが受信された時刻を記録
        if abs(msg.twist.angular.z) > 0.01:
            self.last_yaw_command_time = current_time
        
        # ホバリングコマンドの検出（linear.z=0.0で他の値も0の場合）
        if (abs(msg.twist.linear.x) < 0.01 and abs(msg.twist.linear.y) < 0.01 and 
            abs(msg.twist.linear.z) < 0.01 and abs(msg.twist.angular.x) < 0.01 and 
            abs(msg.twist.angular.y) < 0.01 and abs(msg.twist.angular.z) < 0.01):
            # ホバリングコマンドとして処理
            self.is_hovering = True
            self.get_logger().info("Hover command detected - entering hover mode")
        elif self.is_hovering and (abs(msg.twist.linear.x) > 0.01 or abs(msg.twist.linear.y) > 0.01):
            # ホバリング中に水平移動コマンドが来た場合は、ホバリングを維持
            self.get_logger().info("Horizontal movement command received while hovering - maintaining hover mode")
        elif self.is_hovering and abs(msg.twist.angular.z) > 0.01:
            # ホバリング中に回転コマンドが来た場合は、ホバリングを維持
            self.get_logger().info("Rotation command received while hovering - maintaining hover mode")
        else:
            # 通常の制御コマンド（ホバリング以外）
            self.is_hovering = False
        
        # リセットコマンドの検出（特別な値999.0でリセットを識別）
        if (abs(msg.twist.linear.x - 999.0) < 0.1 and abs(msg.twist.linear.y - 999.0) < 0.1 and 
            abs(msg.twist.linear.z - 999.0) < 0.1 and abs(msg.twist.angular.x - 999.0) < 0.1 and 
            abs(msg.twist.angular.y - 999.0) < 0.1 and abs(msg.twist.angular.z - 999.0) < 0.1):
            # リセットコマンドとして処理
            self.get_logger().info("Reset command detected - executing immediate reset")
            self.get_logger().info(
                f"Current position before reset: [{self.position[0]:.2f}, "
                f"{self.position[1]:.2f}, {self.position[2]:.2f}]"
            )
            self.reset_drone()
            return
        
        self.get_logger().info(
            f"Received control command: throttle={self.control_command[0]:.2f}, "
            f"roll={self.control_command[1]:.2f}, pitch={self.control_command[2]:.2f}, "
            f"yaw={self.control_command[3]:.2f}, vx={self.horizontal_velocity_x:.2f}, "
            f"vy={self.horizontal_velocity_y:.2f}, hovering={self.is_hovering}"
        )
        
        # 水平移動コマンドの特別なログ
        if abs(self.horizontal_velocity_x) > 0.01 or abs(self.horizontal_velocity_y) > 0.01:
            hover_status = "while hovering" if self.is_hovering else "in normal mode"
            self.get_logger().info(
                f"Horizontal movement command: X={self.horizontal_velocity_x:.2f}, "
                f"Y={self.horizontal_velocity_y:.2f} ({hover_status})"
            )
        
        # 回転コマンドの特別なログ
        if abs(self.control_command[3]) > 0.01:  # yaw command
            hover_status = "while hovering" if self.is_hovering else "in normal mode"
            self.get_logger().info(f"Rotation command: Yaw={self.control_command[3]:.2f} ({hover_status})")
    
    def _simulation_step(self):
        """シミュレーションステップ"""
        # 制御コマンドの処理
        throttle = self.control_command[0]
        roll_cmd = self.control_command[1]
        pitch_cmd = self.control_command[2]
        yaw_cmd = self.control_command[3]
        
        # ホバリング制御の改善: 重力と釣り合う推力の計算
        hover_thrust = self.mass * self.gravity  # 重力と釣り合う推力 (約9.81N)
        if self.is_hovering:  # ホバリングフラグがTrueの場合のみ
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # 目標高度を現在の高度に設定（初回のみ、かつ適切な高度の場合）
            if self.target_altitude == 0.0:
                if self.position[2] > 2.0:  # 最低2.0m以上の場合
                    self.target_altitude = self.position[2]
                    self.get_logger().info(f"Hover target altitude set to current: {self.target_altitude:.2f}m")
                else:
                    # 地面近くの場合は適切な高度を設定
                    self.target_altitude = 3.0  # デフォルトのホバリング高度を3.0mに下げる
                    self.get_logger().info(f"Hover target altitude set to default: {self.target_altitude:.2f}m")
                
                # PID制御の状態をリセット
                self.altitude_integral = 0.0
                self.altitude_error_prev = 0.0
                self.hover_start_time = current_time
            
            # 高度PID制御
            altitude_error = self.target_altitude - self.position[2]
            
            # ホバリング開始直後（最初の1秒間）は特別な処理
            if current_time - self.hover_start_time < 1.0:
                # 目標高度に近づいたら即座に重力と釣り合う推力に切り替え
                if altitude_error < 0.2:  # 目標高度から0.2m以内の場合
                    thrust = hover_thrust
                    self.get_logger().debug("Hover target reached, switching to hover thrust")
                else:
                    # まだ目標高度に達していない場合は適度な推力
                    thrust = hover_thrust * 1.02  # 2%大きい推力（より保守的）
                self.get_logger().debug(f"Hover stabilization phase: {current_time - self.hover_start_time:.1f}s")
            else:
                # 通常のPID制御
                # 積分項の制限（ウィンドウアップ防止）
                self.altitude_integral += altitude_error * 0.01  # dt = 0.01s
                self.altitude_integral = np.clip(self.altitude_integral, -0.2, 0.2)  # より厳しい制限
                
                altitude_derivative = (altitude_error - self.altitude_error_prev) / 0.01
                
                # PID制御出力（より保守的な制御）
                altitude_pid = (self.altitude_p * altitude_error + 
                              self.altitude_i * self.altitude_integral + 
                              self.altitude_d * altitude_derivative)
                
                # 推力にPID制御を適用
                thrust = hover_thrust + altitude_pid * self.mass
                
                # オーバーシュート防止のための推力制限
                if altitude_error < 0:  # 目標高度を超えた場合
                    # より積極的に推力を下げる
                    thrust = max(thrust, hover_thrust * 0.6)  # 最小60%の推力
                else:
                    # 通常の推力制限
                    thrust = max(thrust, hover_thrust * 0.75)  # 最小75%の推力を保証
                
                thrust = min(thrust, hover_thrust * 1.05)  # 最大105%の推力に制限
                
                self.altitude_error_prev = altitude_error
        else:
            # 通常の制御時は目標高度をリセット
            self.target_altitude = 0.0
            self.altitude_integral = 0.0
            self.altitude_error_prev = 0.0
            self.hover_start_time = 0.0
            # 通常の推力計算（重力を考慮）
            if abs(throttle) < 0.01:  # スロットルがほぼ0の場合
                thrust = 0.0  # 推力0（重力で落下）
            else:
                # より応答性の良い推力計算
                thrust = throttle * self.max_thrust
                # 最小推力の保証（重力の90%以上）
                if thrust > 0:
                    thrust = max(thrust, hover_thrust * 0.9)
        
        # 角度制御 (簡易的なPID制御)
        roll_error = roll_cmd - self.orientation[0]
        pitch_error = pitch_cmd - self.orientation[1]
        yaw_error = yaw_cmd - self.orientation[2]
        
        # 角速度更新
        self.angular_velocity[0] = self.roll_p * roll_error
        self.angular_velocity[1] = self.pitch_p * pitch_error
        self.angular_velocity[2] = self.yaw_p * yaw_error
        
        # 推力ベクトル計算
        thrust_vector = np.array([
            thrust * np.sin(pitch_cmd),
            -thrust * np.sin(roll_cmd),
            thrust * np.cos(roll_cmd) * np.cos(pitch_cmd)
        ])
        
        # 加速度計算 (重力 + 推力)
        acceleration = thrust_vector / self.mass - np.array([0.0, 0.0, self.gravity])
        
        # 速度更新
        self.velocity += acceleration * 0.01  # dt = 0.01s
        
        # 水平移動コマンドのタイムアウトチェック（無効化：明示的な停止まで継続）
        # current_time = self.get_clock().now().nanoseconds / 1e9
        # if current_time - self.last_horizontal_command_time > self.horizontal_command_timeout:
        #     # タイムアウトした場合は水平移動を停止
        #     self.horizontal_velocity_x = 0.0
        #     self.horizontal_velocity_y = 0.0
        
        # ヨー回転コマンドのタイムアウトチェック（無効化：明示的な停止まで継続）
        # if current_time - self.last_yaw_command_time > self.yaw_command_timeout:
        #     # タイムアウトした場合はヨー回転を停止
        #     self.control_command[3] = 0.0  # yaw = 0
        
        # 水平移動コマンドを直接適用（推力に依存しない）
        horizontal_speed = 1.5  # 水平移動の基準速度を1.5m/sに上げる（より応答性を良く）
        self.velocity[0] = self.horizontal_velocity_x * horizontal_speed  # X軸速度
        self.velocity[1] = self.horizontal_velocity_y * horizontal_speed  # Y軸速度
        
        # 位置更新
        self.position += self.velocity * 0.01
        
        # 姿勢更新
        self.orientation += self.angular_velocity * 0.01
        
        # 地面との衝突処理
        if self.position[2] < 0.0:
            self.position[2] = 0.0
            self.velocity[2] = 0.0
        
        # 位置・姿勢パブリッシュ
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = self.position[0]
        pose_msg.pose.position.y = self.position[1]
        pose_msg.pose.position.z = self.position[2]
        
        # クォータニオン計算 (簡易版)
        import math
        cy = math.cos(self.orientation[2] * 0.5)
        sy = math.sin(self.orientation[2] * 0.5)
        cp = math.cos(self.orientation[1] * 0.5)
        sp = math.sin(self.orientation[1] * 0.5)
        cr = math.cos(self.orientation[0] * 0.5)
        sr = math.sin(self.orientation[0] * 0.5)
        
        pose_msg.pose.orientation.w = cy * cp * cr + sy * sp * sr
        pose_msg.pose.orientation.x = cy * cp * sr - sy * sp * cr
        pose_msg.pose.orientation.y = sy * cp * sr + cy * sp * cr
        pose_msg.pose.orientation.z = sy * cp * cr - cy * sp * sr
        
        self.pose_pub.publish(pose_msg)
        
        # 速度パブリッシュ
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "world"
        twist_msg.twist.linear.x = self.velocity[0]
        twist_msg.twist.linear.y = self.velocity[1]
        twist_msg.twist.linear.z = self.velocity[2]
        twist_msg.twist.angular.x = self.angular_velocity[0]
        twist_msg.twist.angular.y = self.angular_velocity[1]
        twist_msg.twist.angular.z = self.angular_velocity[2]
        
        self.twist_pub.publish(twist_msg)
        
        # IMUデータパブリッシュ
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.linear_acceleration.x = 0.0  # 簡易版
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = -self.gravity
        imu_msg.angular_velocity.x = self.angular_velocity[0]
        imu_msg.angular_velocity.y = self.angular_velocity[1]
        imu_msg.angular_velocity.z = self.angular_velocity[2]
        
        self.imu_pub.publish(imu_msg)
        
        # 推力情報をパブリッシュ
        thrust_msg = Float32()
        thrust_msg.data = thrust
        self.thrust_pub.publish(thrust_msg)
        
        # 状態ログ (1秒に1回)
        if int(time.time() * 100) % 100 == 0:
            hover_info = ""
            if self.is_hovering:
                altitude_error = self.target_altitude - self.position[2]
                hover_info = (
                    f" | Hover: target={self.target_altitude:.2f}m, "
                    f"error={altitude_error:.2f}m, thrust={thrust:.2f}N"
                )
            
            self.get_logger().info(
                f"Position: [{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}] "
                f"Velocity: [{self.velocity[0]:.2f}, {self.velocity[1]:.2f}, {self.velocity[2]:.2f}] "
                f"Throttle: {throttle:.3f}"
                f"{hover_info}"
            )


def main():
    rclpy.init()
    node = SimpleDroneSimulator()
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
        except Exception:
            # 既にシャットダウンされている場合は無視
            pass


if __name__ == '__main__':
    main() 