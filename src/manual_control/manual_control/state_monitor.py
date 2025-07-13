#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
import math

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu


class StateMonitorNode(Node):
    """
    ドローンの状態を監視・表示するノード
    """
    
    def __init__(self):
        super().__init__('state_monitor_node')
        
        # QoS設定
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # パラメータ取得
        self.pose_topic = self.declare_parameter('pose_topic', '/drone/pose').value
        self.twist_topic = self.declare_parameter('twist_topic', '/drone/twist').value
        self.imu_topic = self.declare_parameter('imu_topic', '/imu/data').value
        self.update_rate = self.declare_parameter('update_rate', 1.0).value
        
        # 状態変数
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.angular_velocity = [0.0, 0.0, 0.0]
        
        # サブスクライバー設定
        self._setup_subscribers()
        
        # 表示タイマー
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self._display_state)
        
        self.get_logger().info("State Monitor Node initialized")
    
    def _setup_subscribers(self):
        """サブスクライバー設定"""
        # 位置・姿勢サブスクライバー
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self._pose_callback,
            self.qos_profile
        )
        
        # 速度サブスクライバー
        self.twist_sub = self.create_subscription(
            TwistStamped,
            self.twist_topic,
            self._twist_callback,
            self.qos_profile
        )
        
        # IMUサブスクライバー
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self._imu_callback,
            self.qos_profile
        )
    
    def _pose_callback(self, msg: PoseStamped):
        """位置・姿勢コールバック"""
        self.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        
        # クォータニオンからオイラー角に変換
        q = msg.pose.orientation
        self.orientation = self._quaternion_to_euler(q.x, q.y, q.z, q.w)
    
    def _twist_callback(self, msg: TwistStamped):
        """速度コールバック"""
        self.velocity = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        self.angular_velocity = [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
    
    def _imu_callback(self, msg: Imu):
        """IMUコールバック"""
        # IMUデータは現在使用していないが、必要に応じて拡張可能
        pass
    
    def _quaternion_to_euler(self, x, y, z, w):
        """クォータニオンからオイラー角に変換"""
        # ロール (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # ピッチ (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # ヨー (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]
    
    def _display_state(self):
        """状態表示"""
        # 位置情報
        pos_str = f"Position: X={self.position[0]:6.2f} Y={self.position[1]:6.2f} Z={self.position[2]:6.2f}"
        
        # 速度情報
        vel_str = f"Velocity: X={self.velocity[0]:6.2f} Y={self.velocity[1]:6.2f} Z={self.velocity[2]:6.2f}"
        
        # 姿勢情報（度に変換）
        roll_deg = math.degrees(self.orientation[0])
        pitch_deg = math.degrees(self.orientation[1])
        yaw_deg = math.degrees(self.orientation[2])
        att_str = f"Attitude: Roll={roll_deg:6.1f}° Pitch={pitch_deg:6.1f}° Yaw={yaw_deg:6.1f}°"
        
        # 角速度情報
        ang_vel_str = (
            f"AngVel: X={self.angular_velocity[0]:6.2f} "
            f"Y={self.angular_velocity[1]:6.2f} Z={self.angular_velocity[2]:6.2f}"
        )
        
        # 合計速度
        total_velocity = math.sqrt(sum(v**2 for v in self.velocity))
        total_ang_velocity = math.sqrt(sum(w**2 for w in self.angular_velocity))
        
        # 表示
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"🚁 Drone State Monitor")
        self.get_logger().info("=" * 60)
        self.get_logger().info(pos_str)
        self.get_logger().info(vel_str)
        self.get_logger().info(att_str)
        self.get_logger().info(ang_vel_str)
        self.get_logger().info(f"Total Velocity: {total_velocity:.2f} m/s")
        self.get_logger().info(f"Total Angular Velocity: {total_ang_velocity:.2f} rad/s")
        self.get_logger().info("=" * 60)


def main() -> None:
    """メイン関数"""
    rclpy.init()
    node = StateMonitorNode()
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