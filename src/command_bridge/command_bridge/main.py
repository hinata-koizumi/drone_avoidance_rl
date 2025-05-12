#!/usr/bin/env python3
"""
command_bridge.py – 内蔵ファン 2 基の PWM を PX4 ActuatorServos ch 0-1 に送信
外周ロータ (motors) とは完全に独立するため、PX4 PID と競合しない
"""
import math

import rclpy
from rclpy.node import Node

from drone_msgs.msg import DroneControlCommand as _DroneControlCommand
from px4_msgs.msg import ActuatorServos  # ★変更点

_MOTOR_MIN, _MOTOR_MAX = 0.0, 1.0        # throttle ratio 0–1
_PWM_MIN,   _PWM_MAX   = 900.0, 2100.0   # PX4 標準 PWM [µs]


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _ratio_to_pwm(r: float) -> float:
    """0–1 → 900–2100 µs へ線形変換"""
    return _PWM_MIN + r * (_PWM_MAX - _PWM_MIN)


class CommandBridge(Node):
    def __init__(self) -> None:
        super().__init__("command_bridge")
        self.sub = self.create_subscription(
            _DroneControlCommand, "/drone/inner_propeller_cmd", self._cb, 10
        )
        self.pub = self.create_publisher(ActuatorServos, "/fmu/in/actuator_servos", 10)

    # ---------- callback ----------
    def _cb(self, cmd: '_DroneControlCommand') -> None:
        p1 = _clamp(cmd.throttle1, _MOTOR_MIN, _MOTOR_MAX)
        p2 = _clamp(cmd.throttle2, _MOTOR_MIN, _MOTOR_MAX)
        if any(math.isnan(x) for x in (p1, p2)):
            self.get_logger().error("NaN in throttle – skipped")
            return

        msg = ActuatorServos()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        nan = math.nan
        # ch 0-1 = inner fans, 2-7 = untouched
        msg.control = [
            _ratio_to_pwm(p1), _ratio_to_pwm(p2),
            nan, nan, nan, nan, nan, nan
        ]
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    rclpy.spin(CommandBridge())
    rclpy.shutdown()


if __name__ == "__main__":
    main() 