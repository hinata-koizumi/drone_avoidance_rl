#!/usr/bin/env python3
"""
outer_motor_bridge.py – PX4 ActuatorMotors (ch0-3) を
std_msgs/Float32MultiArray で /drone/outer_motor_pwm へ転送。
配列サイズ 4 なので全ロータ PWM を欠落なく記録できる。
"""
import rclpy
from rclpy.node import Node
from px4_msgs.msg import ActuatorMotors
from std_msgs.msg import Float32MultiArray

class OuterMotorBridge(Node):
    def __init__(self):
        super().__init__("outer_motor_bridge")
        self.sub = self.create_subscription(
            ActuatorMotors,
            "/fmu/out/actuator_motors",
            self._cb,
            10,
        )
        self.pub = self.create_publisher(
            Float32MultiArray,
            "/drone/outer_motor_pwm",
            10,
        )
        self.declare_parameter("pwm_min", 0.0)
        self.declare_parameter("pwm_max", 1.0)

    # ---------- callback ----------
    def _cb(self, msg: ActuatorMotors) -> None:
        pwm_min = self.get_parameter("pwm_min").value
        pwm_max = self.get_parameter("pwm_max").value
        out = Float32MultiArray()
        out.data = [
            max(pwm_min, min(pwm_max, msg.control[i]))
            for i in range(4)            # ch0-3 = 外周ロータ
        ]
        self.pub.publish(out)

def main() -> None:
    rclpy.init()
    rclpy.spin(OuterMotorBridge())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
