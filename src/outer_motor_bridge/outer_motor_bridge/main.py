#!/usr/bin/env python3
"""
outer_motor_bridge.py – PX4 ActuatorMotors (ch0-3) を
std_msgs/Float32MultiArray で /drone/outer_motor_pwm へ転送。
配列サイズ 4 なので全ロータ PWM を欠落なく記録できる。
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor

from px4_msgs.msg import ActuatorMotors  # type: ignore


class OuterMotorBridge(Node):
    def __init__(self) -> None:
        super().__init__("outer_motor_bridge")
        self.declare_parameter("input_topic", "/fmu/out/actuator_motors")
        self.declare_parameter("output_topic", "/drone/outer_motor_pwm")
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_reliability", "reliable")
        self.declare_parameter("qos_history", "keep_last")
        self.declare_parameter("log_level", "debug")
        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
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
            ActuatorMotors,
            input_topic,
            self._cb,
            qos_profile,
        )
        self.pub = self.create_publisher(
            Float32MultiArray,
            output_topic,
            qos_profile,
        )
        self.declare_parameter("pwm_min", 0.0)
        self.declare_parameter("pwm_max", 1.0)
        self.log_level = log_level
        if log_level == "debug":
            self.get_logger().debug(f"Subscribed to: {input_topic}, Publishing to: {output_topic}, QoS: depth={qos_depth}, reliability={qos_reliability}, history={qos_history}")

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