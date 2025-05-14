#!/usr/bin/env python3
"""
angle_bridge.py – 内側可変プロペラ角度を Gazebo JointController へ直接送信
Topic : /servo/fan1_tilt , /servo/fan2_tilt   (std_msgs/Float64, 単位 rad)
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from drone_msgs.msg import DroneControlCommand as _DroneControlCommand  # type: ignore

_MIN_DEG, _MAX_DEG = -30.0, 30.0              # 制御範囲 [deg]
_DEG2RAD = math.pi / 180.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class AngleBridge(Node):
    def __init__(self) -> None:
        super().__init__("angle_bridge")
        self.sub = self.create_subscription(
            _DroneControlCommand,
            "/drone/inner_propeller_cmd",
            self._cb,
            10,
        )
        self.pub1 = self.create_publisher(Float64, "/servo/fan1_tilt", 10)
        self.pub2 = self.create_publisher(Float64, "/servo/fan2_tilt", 10)

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
    rclpy.spin(AngleBridge())
    rclpy.shutdown()


if __name__ == "__main__":
    main() 