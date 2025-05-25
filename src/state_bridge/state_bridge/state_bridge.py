#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor

def main():
    rclpy.init()
    node = StateBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

class StateBridgeNode(Node):
    def __init__(self):
        super().__init__('state_bridge_node')
        self.declare_parameter('input_topic', '/px4/odom')
        self.declare_parameter('output_topic', '/drone/state')
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('qos_reliability', 'reliable')
        self.declare_parameter('qos_history', 'keep_last')
        self.declare_parameter('log_level', 'debug')
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        qos_depth = self.get_parameter('qos_depth').get_parameter_value().integer_value
        qos_reliability = self.get_parameter('qos_reliability').get_parameter_value().string_value
        qos_history = self.get_parameter('qos_history').get_parameter_value().string_value
        log_level = self.get_parameter('log_level').get_parameter_value().string_value
        reliability = ReliabilityPolicy.RELIABLE if qos_reliability == 'reliable' else ReliabilityPolicy.BEST_EFFORT
        history = HistoryPolicy.KEEP_LAST if qos_history == 'keep_last' else HistoryPolicy.KEEP_ALL
        qos_profile = QoSProfile(
            depth=qos_depth,
            reliability=reliability,
            history=history
        )
        # サブスクライバ・パブリッシャ雛形
        self.sub = self.create_subscription(
            type(None),  # TODO: 適切なmsg型に置換
            input_topic,
            self._cb,
            qos_profile,
        )
        self.pub = self.create_publisher(
            type(None),  # TODO: 適切なmsg型に置換
            output_topic,
            qos_profile,
        )
        self.log_level = log_level
        if log_level == 'debug':
            self.get_logger().debug(f"Subscribed to: {input_topic}, Publishing to: {output_topic}, QoS: depth={qos_depth}, reliability={qos_reliability}, history={qos_history}")
    def _cb(self, msg):
        pass  # TODO: 実装

if __name__ == "__main__":
    main() 