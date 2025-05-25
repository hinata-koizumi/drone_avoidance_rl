#!/usr/bin/env python3
"""
angle_bridge.py – 内側可変プロペラ角度を Gazebo JointController へ直接送信
Topic : /servo/fan1_tilt , /servo/fan2_tilt   (std_msgs/Float64, 単位 rad)
"""
import math
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64
from drone_msgs.msg import DroneControlCommand as _DroneControlCommand  # type: ignore
from src.common.bridge_base import BridgeBase
from src.common.utils import clamp

_MIN_DEG, _MAX_DEG = -30.0, 30.0              # 制御範囲 [deg]
_DEG2RAD = math.pi / 180.0

class AngleBridge(BridgeBase):
    def __init__(self) -> None:
        super().__init__("angle_bridge", {
            "cmd_topic": "/drone/inner_propeller_cmd",
            "fan1_topic": "/servo/fan1_tilt",
            "fan2_topic": "/servo/fan2_tilt",
            "qos_depth": 10,
            "qos_reliability": "reliable",
            "qos_history": "keep_last",
            "log_level": "debug"
        })
        cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value
        fan1_topic = self.get_parameter("fan1_topic").get_parameter_value().string_value
        fan2_topic = self.get_parameter("fan2_topic").get_parameter_value().string_value
        self.sub = self.create_subscription(
            _DroneControlCommand,
            cmd_topic,
            self._cb,
            self.qos_profile,
        )
        self.pub1 = self.create_publisher(Float64, fan1_topic, self.qos_profile)
        self.pub2 = self.create_publisher(Float64, fan2_topic, self.qos_profile)
        if self.log_level == "debug":
            self.get_logger().debug(f"Subscribed to: {cmd_topic}, Publishing to: {fan1_topic}, {fan2_topic}, QoS: {self.qos_profile}")

    def _cb(self, cmd: _DroneControlCommand) -> None:
        ang1_deg = clamp(cmd.angle1, _MIN_DEG, _MAX_DEG)
        ang2_deg = clamp(cmd.angle2, _MIN_DEG, _MAX_DEG)
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