#!/usr/bin/env python3
"""
angle_bridge.py – 内側可変プロペラ角度を Gazebo JointController へ直接送信
Topic : /servo/fan1_tilt , /servo/fan2_tilt   (std_msgs/Float64, 単位 rad)
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor

from drone_msgs.msg import DroneControlCommand as _DroneControlCommand  # type: ignore

_MIN_DEG, _MAX_DEG = -30.0, 30.0              # 制御範囲 [deg]
_DEG2RAD = math.pi / 180.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class AngleBridge(Node):
    def __init__(self) -> None:
        super().__init__("angle_bridge")
        self.declare_parameter("cmd_topic", "/drone/inner_propeller_cmd")
        self.declare_parameter("fan1_topic", "/servo/fan1_tilt")
        self.declare_parameter("fan2_topic", "/servo/fan2_tilt")
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_reliability", "reliable")
        self.declare_parameter("qos_history", "keep_last")
        self.declare_parameter("log_level", "debug")
        cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value
        fan1_topic = self.get_parameter("fan1_topic").get_parameter_value().string_value
        fan2_topic = self.get_parameter("fan2_topic").get_parameter_value().string_value
        qos_depth = self.get_parameter("qos_depth").get_parameter_value().integer_value
        qos_reliability = self.get_parameter("qos_reliability").get_parameter_value().string_value
        qos_history = self.get_parameter("qos_history").get_parameter_value().string_value
        log_level = self.get_parameter("log_level").get_parameter_value().string_value
        reliability = ReliabilityPolicy.RELIABLE if qos_reliability == "reliable" else ReliabilityPolicy.BEST_EFFORT
        history = HistoryPolicy.KEEP_LAST if qos_history == "keep_last" else HistoryPolicy.KEEP_ALL
        qos_profile = QoSProfile(
            depth=qos_depth,
            reliability=reliability,
            history=history
        )
        self.sub = self.create_subscription(
            _DroneControlCommand,
            cmd_topic,
            self._cb,
            qos_profile,
        )
        self.pub1 = self.create_publisher(Float64, fan1_topic, qos_profile)
        self.pub2 = self.create_publisher(Float64, fan2_topic, qos_profile)
        self.log_level = log_level
        if log_level == "debug":
            self.get_logger().debug(f"Subscribed to: {cmd_topic}, Publishing to: {fan1_topic}, {fan2_topic}, QoS: depth={qos_depth}, reliability={qos_reliability}, history={qos_history}")

    # ---------- callback ----------
    def _cb(self, cmd: _DroneControlCommand) -> None:
        ang1_deg = _clamp(cmd.angle1, _MIN_DEG, _MAX_DEG)
        ang2_deg = _clamp(cmd.angle2, _MIN_DEG, _MAX_DEG)

        msg1 = Float64()
        msg1.data = ang1_deg * _DEG2RAD
        msg2 = Float64()
        msg2.data = ang2_deg * _DEG2RAD
        self.pub1.publish(msg1)
        self.pub2.publish(msg2)


def main() -> None:
    rclpy.init()
    node = AngleBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main() 