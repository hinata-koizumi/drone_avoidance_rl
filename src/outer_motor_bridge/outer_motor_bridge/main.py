#!/usr/bin/env python3
"""
outer_motor_bridge.py – PX4 ActuatorMotors (ch0-3) を
std_msgs/Float32MultiArray で /drone/outer_motor_pwm へ転送。
配列サイズ 4 なので全ロータ PWM を欠落なく記録できる。
"""
import rclpy
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import ActuatorMotors  # type: ignore
from rclpy.executors import MultiThreadedExecutor
from src.common.bridge_base import BridgeBase


class OuterMotorBridge(BridgeBase):
    def __init__(self) -> None:
        super().__init__("outer_motor_bridge", {
            "input_topic": "/fmu/out/actuator_motors",
            "output_topic": "/drone/outer_motor_pwm",
            "qos_depth": 10,
            "qos_reliability": "reliable",
            "qos_history": "keep_last",
            "log_level": "debug",
            "pwm_min": 0.0,
            "pwm_max": 1.0
        })
        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.sub = self.create_subscription(
            ActuatorMotors,
            input_topic,
            self._cb,
            self.qos_profile,
        )
        self.pub = self.create_publisher(
            Float32MultiArray,
            output_topic,
            self.qos_profile,
        )
        if self.log_level == "debug":
            self.get_logger().debug(
                f"Subscribed to: {input_topic}, Publishing to: {output_topic}, "
                f"QoS: {self.qos_profile}"
            )

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
    node = OuterMotorBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main() 